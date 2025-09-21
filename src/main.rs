use rppal::uart::{Parity, Uart};
use std::io::{self, Write};
use std::thread::sleep;
use std::time::Duration;

const BAUD_RATE: u32 = 115200;
const DATA_BITS: u8 = 8;
const STOP_BITS: u8 = 1;
const DEVICE_ADDRESS: u8 = 0x01;

const READ_MULTIPLE_REGISTERS: u8 = 0x23;
const WRITE_MULTIPLE_REGISTERS: u8 = 0x16;

const READ_INTEGER: u8 = 0xA1;
const READ_FLOAT: u8 = 0xA2;
const READ_STRING: u8 = 0xA3;
const WRITE_INTEGER: u8 = 0xB1;
const WRITE_FLOAT: u8 = 0xB2;
const WRITE_STRING: u8 = 0xB3;

const CRC_TABLE: [u16; 256] = [
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241, 0xC601, 0x06C0, 0x0780, 0xC741,
    0x0500, 0xC5C1, 0xC481, 0x0440, 0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841, 0xD801, 0x18C0, 0x1980, 0xD941,
    0x1B00, 0xDBC1, 0xDA81, 0x1A40, 0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641, 0xD201, 0x12C0, 0x1380, 0xD341,
    0x1100, 0xD1C1, 0xD081, 0x1040, 0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441, 0x3C00, 0xFCC1, 0xFD81, 0x3D40,
    0xFF01, 0x3FC0, 0x3E80, 0xFE41, 0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41, 0xEE01, 0x2EC0, 0x2F80, 0xEF41,
    0x2D00, 0xEDC1, 0xEC81, 0x2C40, 0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041, 0xA001, 0x60C0, 0x6180, 0xA141,
    0x6300, 0xA3C1, 0xA281, 0x6240, 0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41, 0xAA01, 0x6AC0, 0x6B80, 0xAB41,
    0x6900, 0xA9C1, 0xA881, 0x6840, 0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40, 0xB401, 0x74C0, 0x7580, 0xB541,
    0x7700, 0xB7C1, 0xB681, 0x7640, 0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241, 0x9601, 0x56C0, 0x5780, 0x9741,
    0x5500, 0x95C1, 0x9481, 0x5440, 0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841, 0x8801, 0x48C0, 0x4980, 0x8941,
    0x4B00, 0x8BC1, 0x8A81, 0x4A40, 0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641, 0x8201, 0x42C0, 0x4380, 0x8341,
    0x4100, 0x81C1, 0x8081, 0x4040,
];

fn crc16_byte(crc: u16, data: u8) -> u16 {
    ((crc & 0xFF00) >> 8) ^ CRC_TABLE[((crc & 0x00FF) ^ (data as u16)) as usize]
}

fn calculate_crc16(message: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &byte in message {
        crc = crc16_byte(crc, byte);
    }
    crc
}

fn calculate_transmission_time(bytes: usize, baud_rate: u32) -> Duration {
    let bits_per_byte = 1 + DATA_BITS + STOP_BITS;
    let total_bits = bytes * bits_per_byte as usize;
    let transmission_time_us = (total_bits as u64 * 1_000_000) / baud_rate as u64;
    Duration::from_micros(transmission_time_us)
}

fn build_modbus_message(function_code: u8, mut data: Vec<u8>) -> Vec<u8> {
    data.extend_from_slice(&[2, 5, 2, 6]);

    let mut message = vec![DEVICE_ADDRESS, function_code];
    message.extend_from_slice(&data);

    let crc = calculate_crc16(&message);

    message.push((crc & 0xFF) as u8);
    message.push((crc >> 8) as u8);

    message
}

fn print_hex_message(label: &str, message: &[u8]) {
    print!("{}: ", label);
    for byte in message {
        print!("{:02X} ", byte);
    }
    println!();
}

fn send_modbus_message(message: &[u8]) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
    let mut uart = Uart::with_path("/dev/ttyS0", BAUD_RATE, Parity::None, DATA_BITS, STOP_BITS)?;
    uart.set_read_mode(1, Duration::from_secs(1))?;

    print_hex_message("Sending", message);

    uart.write(message)?;

    let transmission_time = calculate_transmission_time(message.len(), BAUD_RATE);
    let wait_time = transmission_time + Duration::from_millis(50);
    sleep(wait_time);

    let mut response = vec![0; 255];
    let bytes_read = uart.read(&mut response)?;
    response.truncate(bytes_read);

    if !response.is_empty() {
        print_hex_message("Received", &response);

        if response.len() >= 2 {
            let message_part = &response[..response.len() - 2];
            let received_crc =
                u16::from_le_bytes([response[response.len() - 2], response[response.len() - 1]]);
            let calculated_crc = calculate_crc16(message_part);

            if received_crc == calculated_crc {
                println!("CRC verification: OK");
            } else {
                println!(
                    "CRC verification: FAILED (received: 0x{:04X}, calculated: 0x{:04X})",
                    received_crc, calculated_crc
                );
            }
        }

        if response.len() >= 2 && (response[1] & 0x80) != 0 {
            let error_code = if response.len() >= 3 { response[2] } else { 0 };
            println!("ERROR: Exception code 0x{:02X}", error_code);
        }
    } else {
        println!("No response received");
    }

    drop(uart);

    Ok(response)
}

fn request_integer() -> Result<(), Box<dyn std::error::Error>> {
    println!("\n=== Requesting Integer Data ===");
    let message = build_modbus_message(READ_MULTIPLE_REGISTERS, vec![READ_INTEGER]);
    send_modbus_message(&message)?;
    Ok(())
}

fn request_float() -> Result<(), Box<dyn std::error::Error>> {
    println!("\n=== Requesting Float Data ===");
    let message = build_modbus_message(READ_MULTIPLE_REGISTERS, vec![READ_FLOAT]);
    send_modbus_message(&message)?;
    Ok(())
}

fn request_string() -> Result<(), Box<dyn std::error::Error>> {
    println!("\n=== Requesting String Data ===");
    let message = build_modbus_message(READ_MULTIPLE_REGISTERS, vec![READ_STRING]);
    send_modbus_message(&message)?;
    Ok(())
}

fn send_integer() -> Result<(), Box<dyn std::error::Error>> {
    println!("\n=== Sending Integer Data ===");
    print!("Enter integer value: ");
    io::stdout().flush()?;

    let mut input = String::new();
    io::stdin().read_line(&mut input)?;
    let value: i32 = input.trim().parse()?;

    let mut data = vec![WRITE_INTEGER];
    data.extend_from_slice(&value.to_be_bytes());
    let message = build_modbus_message(WRITE_MULTIPLE_REGISTERS, data);
    send_modbus_message(&message)?;
    Ok(())
}

fn send_float() -> Result<(), Box<dyn std::error::Error>> {
    println!("\n=== Sending Float Data ===");
    print!("Enter float value: ");
    io::stdout().flush()?;

    let mut input = String::new();
    io::stdin().read_line(&mut input)?;
    let value: f32 = input.trim().parse()?;

    let mut data = vec![WRITE_FLOAT];
    data.extend_from_slice(&value.to_be_bytes());
    let message = build_modbus_message(WRITE_MULTIPLE_REGISTERS, data);
    send_modbus_message(&message)?;
    Ok(())
}

fn send_string() -> Result<(), Box<dyn std::error::Error>> {
    println!("\n=== Sending String Data ===");
    print!("Enter string: ");
    io::stdout().flush()?;

    let mut input = String::new();
    io::stdin().read_line(&mut input)?;
    let string_data = input.trim();

    let mut data = vec![WRITE_STRING];
    data.push(string_data.len() as u8);
    data.extend_from_slice(string_data.as_bytes());
    let message = build_modbus_message(WRITE_MULTIPLE_REGISTERS, data);
    send_modbus_message(&message)?;
    Ok(())
}

fn display_menu() {
    println!("\n==================== MODBUS RTU MENU ====================");
    println!("1. Request Integer Data (0x23, 0xA1)");
    println!("2. Request Float Data (0x23, 0xA2)");
    println!("3. Request String Data (0x23, 0xA3)");
    println!("4. Send Integer Data (0x16, 0xB1)");
    println!("5. Send Float Data (0x16, 0xB2)");
    println!("6. Send String Data (0x16, 0xB3)");
    println!("7. Exit");
    println!("=========================================================");
    print!("Select an option (1-7): ");
    io::stdout().flush().unwrap();
}

fn main() {
    println!("Modbus RTU UART Communication System");
    println!(
        "UART Configuration: {} baud, {} data bits, {} stop bit, no parity",
        BAUD_RATE, DATA_BITS, STOP_BITS
    );
    println!("Device Address: 0x{:02X}", DEVICE_ADDRESS);

    loop {
        display_menu();

        let mut input = String::new();
        match io::stdin().read_line(&mut input) {
            Ok(_) => match input.trim() {
                "1" => {
                    if let Err(e) = request_integer() {
                        println!("Error: {}", e);
                    }
                }
                "2" => {
                    if let Err(e) = request_float() {
                        println!("Error: {}", e);
                    }
                }
                "3" => {
                    if let Err(e) = request_string() {
                        println!("Error: {}", e);
                    }
                }
                "4" => {
                    if let Err(e) = send_integer() {
                        println!("Error: {}", e);
                    }
                }
                "5" => {
                    if let Err(e) = send_float() {
                        println!("Error: {}", e);
                    }
                }
                "6" => {
                    if let Err(e) = send_string() {
                        println!("Error: {}", e);
                    }
                }
                "7" => {
                    println!("Exiting...");
                    break;
                }
                _ => {
                    println!("Invalid option. Please select 1-7.");
                }
            },
            Err(e) => {
                println!("Error reading input: {}", e);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc16_calculation() {
        let test_data = [0x01, 0x23, 0xA1];
        let crc = calculate_crc16(&test_data);
        assert_ne!(crc, 0);
    }

    #[test]
    fn test_message_building() {
        let message = build_modbus_message(READ_MULTIPLE_REGISTERS, vec![READ_INTEGER]);

        assert_eq!(message[0], DEVICE_ADDRESS);
        assert_eq!(message[1], READ_MULTIPLE_REGISTERS);
        assert_eq!(message[2], READ_INTEGER);

        assert_eq!(message[3], 0x32);
        assert_eq!(message[4], 0x35);
        assert_eq!(message[5], 0x32);
        assert_eq!(message[6], 0x36);
        assert_eq!(message.len(), 9);
    }

    #[test]
    fn test_transmission_time_calculation() {
        let time = calculate_transmission_time(10, 115200);
        assert!(time > Duration::from_micros(0));
    }
}
