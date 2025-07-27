import smbus2

def poll_i2c_register(i2c_bus, i2c_address, register_address, expected_value):
    """
    Polls an 8-bit I2C register at a 16-bit address and checks each bit against an expected value.

    Args:
        i2c_bus (int): The I2C bus number (e.g., 2).
        i2c_address (int): The I2C device address (e.g., 0x29).
        register_address (int): The 16-bit register address to poll (e.g., 0x1234).
        expected_value (int): The expected 8-bit value of the register (e.g., 0x00).
    """
    try:
        # Open the I2C bus
        bus = smbus2.SMBus(i2c_bus)

        # Write the 16-bit register address (split into high and low bytes)
        high_byte = (register_address >> 8) & 0xFF
        low_byte = register_address & 0xFF
        bus.write_i2c_block_data(i2c_address, high_byte, [low_byte])

        # Read the 8-bit register value
        register_value = bus.read_byte(i2c_address)

        print(f"Polling I2C bus {i2c_bus}, address {hex(i2c_address)}, register {hex(register_address)}...")
        print(f"Register value: {hex(register_value)}")
        print(f"Expected value: {hex(expected_value)}")

        # Compare each bit
        for bit in range(8):
            mask = 1 << bit
            register_bit = register_value & mask
            expected_bit = expected_value & mask

            if register_bit != expected_bit:
                print(f"Error: Bit {bit} is not as expected. Register bit: {register_bit}, Expected bit: {expected_bit}")

    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Close the I2C bus
        bus.close()

# Example usage
if __name__ == "__main__":
    i2c_bus = 2          # I2C bus number
    i2c_address = 0x29   # I2C device address
    register_address = 0x1234  # 16-bit register address
    expected_value = 0x00  # Expected 8-bit value

    poll_i2c_register(i2c_bus, i2c_address, register_address, expected_value)