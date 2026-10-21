#!/usr/bin/env python3
"""
Assemble PS2000 Init Script JSON to C include file.

Reconstructs the binary format expected by sub_280C0 parser.
The output is a C include file containing a byte array that can
be embedded in the library:

    static const uint8_t _ps2000_init_script[] = {
    #include "init_script_data.inc"
    };

Usage:
    python3 init_tbl_asm.py init_script.json > init_script_data.inc
    python3 init_tbl_asm.py init_script.json --out init_script_data.inc
    python3 init_tbl_asm.py init_script.json --var-name my_script
"""
import sys
import json
import struct
import argparse

# ==========================================================================
# IO COMMAND TABLE (reverse mapping: name -> opcode)
# ==========================================================================

IO_COMMANDS_REVERSE = {
    "IO_COMMAND_READ_XDATA":               0x00,
    "IO_COMMAND_WRITE_XDATA":              0x01,
    "IO_COMMAND_READ_EXTENDED_REGISTER":   0x02,
    "IO_COMMAND_WRITE_EXTENDED_REGISTER":  0x03,
    "IO_COMMAND_READ_I2C":                 0x04,
    "IO_COMMAND_READ_I2C_WITH_STATUS":     0x05,
    "IO_COMMAND_WRITE_I2C":                0x06,
    "IO_COMMAND_WRITE_SPI":                0x07,
    "IO_COMMAND_WRITE_ADC_SPI":            0x08,
    "IO_COMMAND_READ_WRITE_SPI":           0x09,
    "IO_COMMAND_WRITE_I2C_VIA_SPI":        0x0A,
    "IO_COMMAND_READ_I2C_VIA_SPI":         0x0B,
    "IO_COMMAND_READ_WRITE_ADC_SPI":       0x0C,
    "IO_COMMAND_WAIT_FOR_EEPROM_WRITE":    0x0D,
    "IO_COMMAND_WAIT_FOR_FX3_EEPROM_WRITE":0x0E,
    "IO_COMMAND_READ_FPGA_REGISTER":       0x0F,
    "IO_COMMAND_WRITE_FPGA_REGISTER":      0x10,
    "IO_COMMAND_READ_WRITE_FPGA_REGISTER": 0x11,
    "IO_COMMAND_READ_FIFO":                0x12,
    "IO_COMMAND_ABORT_READ_FIFO":          0x13,
    "IO_COMMAND_WRITE_FIFO":               0x14,
    "IO_SET_AUTOIN_LEN":                   0x15,
    "IO_DELAY_MS":                         0x16,
    "IO_READ_EEPROM":                      0x17,
    "IO_WRITE_EEPROM":                     0x18,
    "IO_COMMAND_FX3_USE_GPIO":             0x19,
    "IO_COMMAND_FX3_UNUSE_GPIO":           0x1A,
    "IO_COMMAND_FX3_IS_GPIF_DONE":         0x1B,
    "IO_COMMAND_FX3_GPIO_PWM":             0x1C,
    "IO_COMMAND_USB_WRITE_GPIF_FREQ":      0x1D,
    "IO_COMMAND_FX3_READ_REGISTER":        0x1E,
    "IO_COMMAND_READ_FLASH_PAGE":          0x1F,
    "USB_IO_WRITE_FLASH_PAGE":             0x20,
    "USB_IO_ERASE_FLASH_PAGE":             0x21,
    "USB_IO_SELECT_CSN_SPI":               0x22,
    "IO_COMMAND_CHECK_EMPTY_EEPROM":       0x23,
    "IO_COMMAND_CHECK_FLASH_AVAIL":        0x24,
    "IO_COMMAND_PROTECT_FLASH":            0x25,
    "IO_COMMAND_ERASE_FLASH_PAGE":         0x26,
    "IO_COMMAND_PACKET":                   0x27,
    "FPGA_IO_COMMAND":                     0x28,
    "END":                                 0xFF,
}

# Commands with variable-length data (16-bit length prefix + data)
VARIABLE_COMMANDS = {
    "IO_COMMAND_WRITE_FIFO",
    "IO_WRITE_EEPROM",
    "USB_IO_WRITE_FLASH_PAGE",
    "IO_COMMAND_PACKET",
    "FPGA_IO_COMMAND",
}

# Commands with no payload
NO_PAYLOAD_COMMANDS = {
    "IO_COMMAND_WAIT_FOR_EEPROM_WRITE",
    "IO_COMMAND_WAIT_FOR_FX3_EEPROM_WRITE",
    "IO_COMMAND_ABORT_READ_FIFO",
    "IO_COMMAND_FX3_IS_GPIF_DONE",
    "IO_COMMAND_CHECK_EMPTY_EEPROM",
    "IO_COMMAND_CHECK_FLASH_AVAIL",
    "END",
}

# ==========================================================================
# ASSEMBLER
# ==========================================================================

def parse_hex_or_int(value, default=0):
    """Parse a value that may be hex string or integer."""
    if value is None:
        return default
    if isinstance(value, int):
        return value
    if isinstance(value, str):
        return int(value, 16) if value.startswith("0x") else int(value)
    return default

def assemble(cmds: list) -> bytearray:
    """
    Assemble command list into binary init script.
    
    Args:
        cmds: List of command dictionaries (from disassembler output)
        
    Returns:
        Binary script data
        
    Raises:
        ValueError: If an unknown opcode is encountered
    """
    out = bytearray()
    
    for i, cmd in enumerate(cmds):
        # Skip entries with errors from disassembly
        if "error" in cmd:
            print(f"Warning: Skipping command {i} with error: {cmd['error']}",
                  file=sys.stderr)
            continue
        
        name = cmd.get("op", "")
        
        if name not in IO_COMMANDS_REVERSE:
            raise ValueError(f"Unknown opcode at index {i}: {name}")
        
        op = IO_COMMANDS_REVERSE[name]
        out.append(op)
        
        # END marker terminates the script
        if name == "END":
            break
        
        # Variable-length commands
        if name in VARIABLE_COMMANDS:
            data_hex = cmd.get("data_hex", "")
            data_bytes = bytes.fromhex(data_hex) if data_hex else b""
            
            # Check length fits in 16-bit prefix
            if len(data_bytes) > 0xFFFF:
                raise ValueError(
                    f"Command {i} ({name}): data too long ({len(data_bytes)} bytes)")
            
            out.extend(struct.pack('<H', len(data_bytes)))
            out.extend(data_bytes)
        
        # No-payload commands
        elif name in NO_PAYLOAD_COMMANDS:
            pass
        
        # Fixed-length payload commands
        else:
            addr = parse_hex_or_int(cmd.get("addr"), 0)
            out.append(addr & 0xFF)
            
            if "value" in cmd:
                val = parse_hex_or_int(cmd["value"], 0)
                out.extend(struct.pack('<H', val & 0xFFFF))
            
            if "extra" in cmd:
                extra = parse_hex_or_int(cmd["extra"], 0)
                out.append(extra & 0xFF)
    
    # Ensure script ends with END marker
    if len(out) == 0 or out[-1] != 0xFF:
        out.append(0xFF)
    
    return out

# ==========================================================================
# C INCLUDE FORMATTER
# ==========================================================================

def format_c_include(data: bytearray, 
                     var_name: str = "_ps2000_init_script",
                     bytes_per_line: int = 16) -> str:
    """
    Format binary data as C include file.
    
    Args:
        data: Binary script data
        var_name: C variable name for the array
        bytes_per_line: Number of bytes per line
        
    Returns:
        C include file content
    """
    lines = []
    lines.append(f"/* Auto-generated PS2000 init script ({len(data)} bytes) */")
    lines.append(f"/* Generated by init_tbl_asm.py */")
    lines.append(f"/* Do not edit manually - regenerate from JSON source */")
    lines.append("")
    lines.append(f"static const uint8_t {var_name}[] = {{")
    
    for i in range(0, len(data), bytes_per_line):
        chunk = data[i:i+bytes_per_line]
        hex_vals = ", ".join(f"0x{b:02X}" for b in chunk)
        comma = "," if i + bytes_per_line < len(data) else ""
        
        # Add offset comment for readability
        offset_comment = f"  /* 0x{i:04X} */"
        lines.append(f"    {hex_vals}{comma}{offset_comment}")
    
    lines.append("};")
    lines.append("")
    lines.append(f"static const uint32_t {var_name}_len = {len(data)}U;")
    lines.append("")
    
    return "\n".join(lines)

# ==========================================================================
# MAIN
# ==========================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Assemble PS2000 init script JSON to C include file",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    %(prog)s init_script.json
    %(prog)s init_script.json --out init_script_data.inc
    %(prog)s init_script.json --var-name my_custom_script
        """
    )
    parser.add_argument("input", help="Input JSON file")
    parser.add_argument("--out", "-o", default="-",
                        help="Output C include file (default: stdout)")
    parser.add_argument("--var-name", default="_ps2000_init_script",
                        help="C variable name for the script array")
    parser.add_argument("--bytes-per-line", type=int, default=16,
                        help="Bytes per line in output (default: 16)")
    args = parser.parse_args()
    
    try:
        with open(args.input) as f:
            data = json.load(f)
    except IOError as e:
        print(f"Error reading file: {e}", file=sys.stderr)
        sys.exit(1)
    except json.JSONDecodeError as e:
        print(f"Invalid JSON: {e}", file=sys.stderr)
        sys.exit(1)
    
    # Support both flat list and wrapped format from disassembler
    if isinstance(data, dict) and "commands" in data:
        cmds = data["commands"]
    elif isinstance(data, list):
        cmds = data
    else:
        print("Error: Invalid JSON format. Expected list or "
              "object with 'commands' key.", file=sys.stderr)
        sys.exit(1)
    
    try:
        binary = assemble(cmds)
    except ValueError as e:
        print(f"Assembly error: {e}", file=sys.stderr)
        sys.exit(1)
    
    output = format_c_include(binary, args.var_name, args.bytes_per_line)
    
    if args.out == "-":
        print(output)
    else:
        try:
            with open(args.out, "w") as f:
                f.write(output)
            print(f"Written {len(binary)} bytes to {args.out}", file=sys.stderr)
        except IOError as e:
            print(f"Error writing file: {e}", file=sys.stderr)
            sys.exit(1)

if __name__ == "__main__":
    main()