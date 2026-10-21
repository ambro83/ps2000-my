#!/usr/bin/env python3
"""
Disassemble PS2000 Init Script binary to JSON.

Reconstructs the parsing logic from sub_280C0, which is called from
_ps2000_context_init+C0 and ps2000PreLaunch+F4.

The init script is a sequence of IO commands that configure the MCU,
ADC, FPGA, and other hardware during device initialization.

Binary format:
    [opcode:1][payload:variable]
    
    For fixed-size commands, payload size is determined by opcode.
    For variable-size commands, payload starts with 16-bit length prefix.

Usage:
    python3 init_tbl_disasm.py init_script.bin > init_script.json
    python3 init_tbl_disasm.py init_script.bin --out init_script.json
"""
import sys
import json
import struct
import argparse

# ==========================================================================
# IO COMMAND TABLE
# ==========================================================================
#
# Full list reconstructed from RTTI type names in .rodata:
#   - 0xA64D0..0xA6B40: Original 36 commands
#   - 0xA69D0..0xA6AE0: Additional flash/GPIO commands
#
# Format: opcode -> (name, type, payload_size)
#   payload_size: fixed byte count, or "variable" for length-prefixed data
#
# The numeric prefix in RTTI strings (e.g., "21IO_COMMAND_READ_XDATA")
# is the Itanium ABI name length; the actual identifier follows.

IO_COMMANDS = {
    # XDATA access (FX2/FX3 external memory)
    0x00: ("IO_COMMAND_READ_XDATA",              "read",      3),
    0x01: ("IO_COMMAND_WRITE_XDATA",             "write",     3),
    
    # Extended register access
    0x02: ("IO_COMMAND_READ_EXTENDED_REGISTER",  "read",      3),
    0x03: ("IO_COMMAND_WRITE_EXTENDED_REGISTER", "write",     3),
    
    # I2C bus operations
    0x04: ("IO_COMMAND_READ_I2C",                "read",      4),
    0x05: ("IO_COMMAND_READ_I2C_WITH_STATUS",    "read",      4),
    0x06: ("IO_COMMAND_WRITE_I2C",               "write",     4),
    
    # SPI bus operations
    0x07: ("IO_COMMAND_WRITE_SPI",               "write",     4),
    0x08: ("IO_COMMAND_WRITE_ADC_SPI",           "write",     3),
    0x09: ("IO_COMMAND_READ_WRITE_SPI",          "readwrite", 4),
    
    # I2C via SPI bridge
    0x0A: ("IO_COMMAND_WRITE_I2C_VIA_SPI",       "write",     4),
    0x0B: ("IO_COMMAND_READ_I2C_VIA_SPI",        "read",      4),
    
    # ADC SPI operations
    0x0C: ("IO_COMMAND_READ_WRITE_ADC_SPI",      "readwrite", 3),
    
    # EEPROM synchronization
    0x0D: ("IO_COMMAND_WAIT_FOR_EEPROM_WRITE",   "wait",      0),
    0x0E: ("IO_COMMAND_WAIT_FOR_FX3_EEPROM_WRITE","wait",     0),
    
    # FPGA register access
    0x0F: ("IO_COMMAND_READ_FPGA_REGISTER",      "read",      2),
    0x10: ("IO_COMMAND_WRITE_FPGA_REGISTER",     "write",     3),
    0x11: ("IO_COMMAND_READ_WRITE_FPGA_REGISTER","readwrite", 3),
    
    # FIFO operations
    0x12: ("IO_COMMAND_READ_FIFO",               "read",      4),
    0x13: ("IO_COMMAND_ABORT_READ_FIFO",         "abort",     0),
    0x14: ("IO_COMMAND_WRITE_FIFO",              "write",     "variable"),
    
    # Auto-in and timing
    0x15: ("IO_SET_AUTOIN_LEN",                  "set",       2),
    0x16: ("IO_DELAY_MS",                        "delay",     2),
    
    # EEPROM access
    0x17: ("IO_READ_EEPROM",                     "read",      4),
    0x18: ("IO_WRITE_EEPROM",                    "write",     "variable"),
    
    # FX3 GPIO operations
    0x19: ("IO_COMMAND_FX3_USE_GPIO",            "gpio",      2),
    0x1A: ("IO_COMMAND_FX3_UNUSE_GPIO",          "gpio",      2),
    0x1B: ("IO_COMMAND_FX3_IS_GPIF_DONE",        "status",    0),
    0x1C: ("IO_COMMAND_FX3_GPIO_PWM",            "pwm",       4),
    
    # USB/GPIF frequency
    0x1D: ("IO_COMMAND_USB_WRITE_GPIF_FREQ",     "freq",      4),
    
    # FX3 register access
    0x1E: ("IO_COMMAND_FX3_READ_REGISTER",       "read",      3),
    
    # Flash memory operations
    0x1F: ("IO_COMMAND_READ_FLASH_PAGE",         "read",      4),
    0x20: ("USB_IO_WRITE_FLASH_PAGE",            "write",     "variable"),
    0x21: ("USB_IO_ERASE_FLASH_PAGE",            "erase",     2),
    0x22: ("USB_IO_SELECT_CSN_SPI",              "select",    1),
    
    # EEPROM/Flash status checks
    0x23: ("IO_COMMAND_CHECK_EMPTY_EEPROM",      "check",     0),
    0x24: ("IO_COMMAND_CHECK_FLASH_AVAIL",       "check",     0),
    
    # Flash protection and erase
    0x25: ("IO_COMMAND_PROTECT_FLASH",           "protect",   2),
    0x26: ("IO_COMMAND_ERASE_FLASH_PAGE",        "erase",     2),
    
    # Generic packet and FPGA commands
    0x27: ("IO_COMMAND_PACKET",                  "packet",    "variable"),
    0x28: ("FPGA_IO_COMMAND",                    "fpga",      "variable"),
    
    # End of script marker
    0xFF: ("END",                                "end",       0),
}

# ==========================================================================
# DISASSEMBLER
# ==========================================================================

def disassemble(data: bytes) -> list:
    """
    Disassemble binary init script into command list.
    
    Args:
        data: Raw binary script data
        
    Returns:
        List of command dictionaries
    """
    cmds = []
    pos = 0
    
    while pos < len(data):
        op = data[pos]
        pos += 1
        
        # Unknown opcode - record and stop
        if op not in IO_COMMANDS:
            cmds.append({
                "offset": pos - 1,
                "error": f"unknown_opcode_0x{op:02X}",
                "raw": data[pos-1:min(pos+8, len(data))].hex()
            })
            break
        
        name, cmd_type, payload_size = IO_COMMANDS[op]
        entry = {
            "offset": pos - 1,
            "op": name,
            "type": cmd_type
        }
        
        # END marker terminates the script
        if name == "END":
            cmds.append(entry)
            break
        
        # Variable-length commands: read 16-bit length prefix
        if payload_size == "variable":
            if pos + 2 > len(data):
                entry["error"] = "truncated_length"
                cmds.append(entry)
                break
            
            data_len = struct.unpack_from('<H', data, pos)[0]
            pos += 2
            
            if pos + data_len > len(data):
                entry["error"] = "truncated_data"
                entry["expected_len"] = data_len
                entry["available"] = len(data) - pos
                cmds.append(entry)
                break
            
            entry["data_len"] = data_len
            entry["data_hex"] = data[pos:pos+data_len].hex()
            pos += data_len
        
        # Fixed-length payload
        else:
            if pos + payload_size > len(data):
                entry["error"] = "truncated_payload"
                entry["expected_bytes"] = payload_size
                entry["available"] = len(data) - pos
                cmds.append(entry)
                break
            
            payload = data[pos:pos+payload_size]
            pos += payload_size
            
            # Decode payload fields based on size
            if payload_size >= 1:
                entry["addr"] = f"0x{payload[0]:02X}"
            if payload_size >= 3:
                entry["value"] = f"0x{struct.unpack_from('<H', payload, 1)[0]:04X}"
            if payload_size >= 4:
                entry["extra"] = f"0x{payload[3]:02X}"
        
        cmds.append(entry)
    
    return cmds

# ==========================================================================
# MAIN
# ==========================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Disassemble PS2000 init script binary to JSON",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    %(prog)s init_script.bin
    %(prog)s init_script.bin --out init_script.json
    %(prog)s init_script.bin --stats
        """
    )
    parser.add_argument("input", help="Input binary file")
    parser.add_argument("--out", "-o", default="-",
                        help="Output JSON file (default: stdout)")
    parser.add_argument("--stats", action="store_true",
                        help="Print command statistics")
    args = parser.parse_args()
    
    try:
        with open(args.input, "rb") as f:
            data = f.read()
    except IOError as e:
        print(f"Error reading file: {e}", file=sys.stderr)
        sys.exit(1)
    
    commands = disassemble(data)
    
    # Print statistics if requested
    if args.stats:
        op_counts = {}
        errors = 0
        for cmd in commands:
            if "error" in cmd:
                errors += 1
            else:
                op = cmd["op"]
                op_counts[op] = op_counts.get(op, 0) + 1
        
        print(f"File: {args.input}", file=sys.stderr)
        print(f"Size: {len(data)} bytes", file=sys.stderr)
        print(f"Commands: {len(commands)}", file=sys.stderr)
        print(f"Errors: {errors}", file=sys.stderr)
        print("\nCommand breakdown:", file=sys.stderr)
        for op, count in sorted(op_counts.items(), key=lambda x: -x[1]):
            print(f"  {op}: {count}", file=sys.stderr)
        print("", file=sys.stderr)
    
    # Build result
    result = {
        "file": args.input,
        "size": len(data),
        "command_count": len(commands),
        "commands": commands
    }
    
    output = json.dumps(result, indent=2)
    
    if args.out == "-":
        print(output)
    else:
        try:
            with open(args.out, "w") as f:
                f.write(output)
            print(f"Written to {args.out}", file=sys.stderr)
        except IOError as e:
            print(f"Error writing file: {e}", file=sys.stderr)
            sys.exit(1)

if __name__ == "__main__":
    main()