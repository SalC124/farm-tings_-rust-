/* THIS CODE IS TO POWER AND UNPOWER AN LED AT SPECIFIED INTERVALS

USES GPIO PIN 21, which is the same as PIN 40
 * (dont forget to add the target -> `rustup target add armv7a-none-eabi`)

 * although the [datasheet](https://cs140e.sergio.bz/docs/BCM2837-ARM-Peripherals.pdf) states that address `0x7E000000` is where you should move data to, the actual physical address located at `0x3F00 0000`, so that is what I will be referencing (page 6; '1.2.3 ARM physical addresses')


In order to make PIN 21 OUTPUT, so I must write `001` to bits 3-5, so basically...

|   address   | written bits | what it does |
+-------------+--------------+--------------+
| 0x3F00 0000 | N/A          | base address | <- for ref
+-------------+--------------+--------------+
| 0x3F20 0008 |1<<3;0<<4;0<<5| gpio 21 OUT  |
+-------------+--------------+--------------+
| 0x3F20 001C | 1<<21        | gpio 21 ON   |
+-------------+--------------+--------------+
| 0x3F20 0028 | 1<<21        | gpio 21 OFF  |
+-------------+--------------+--------------+


 * we get the address `0x7E20 0008` from 'Table 6-4 – GPIO Alternate function select register 2' (page 93) and looking that up in the table of registers (page 90; 'Table 6-1 GPIO Register Assignment')

 * we get the address `0x7E20 001C` from 'Table 6-8 – GPIO Output Set Register 0' (page 95). Since 21 is in the range 0..=31, 0 is used and we find the address in the table (page 90; 'Table 6-1 GPIO Register Assignment')
 * * same logic used to find address `0x7E20 0028`: 1..=31 so use 0

*/

#![no_std]
#![no_main]

const BASE_ADDRESS: usize = 0x3F20_0000;

use core::{arch::asm, panic::PanicInfo}; // same as `use core::arch::asm;` + `use core::panic::PanicInfo;`

// makes sure that `_start` is at the beginning of the module, and all code goes after
mod boot {
    use core::arch::global_asm;

    global_asm!(".section .text._start");
    // all code below this is in the section `.text._start`
}

#[no_mangle]
pub extern "C" fn _start() -> ! {
    //core::ptr::write_volatile(0x3F20_0000 as *mut u32, 1 << 12);
    set_mode(21, Pinmode::Output);
    loop {
        digital_write(21, Power::HIGH);
        //core::ptr::write_volatile(0x3F20_001C as *mut u32, 1 << 21);

        // delay of 16384 cycles
        for _ in 1..100000 {
            unsafe {
                asm!("nop");
            }
        }

        digital_write(21, Power::LOW);
        //core::ptr::write_volatile(0x3F20_0028 as *mut u32, 1 << 21);

        // delay of 16384 cycles
        for _ in 1..100000 {
            unsafe {
                asm!("nop");
            }
        }
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}

pub enum Pinmode {
    Input,
    Output,
}

fn set_mode(pin: u8, mode: Pinmode) {
    let register: u8 = pin / 10;
    let bits = (pin % 10) * 3;
    let change_bits: [u8; 3] = [bits + 2, bits + 1, bits];
    let sel_bits = match mode {
        Pinmode::Input => [0, 0, 0],
        Pinmode::Output => [0, 0, 1],
    };
    for (index, &change_bit) in change_bits.iter().enumerate() {
        unsafe {
            core::ptr::write_volatile(
                BASE_ADDRESS.wrapping_add((register * 4).into()) as *mut u32,
                sel_bits[index] << change_bit,
            )
        };
    }
}

pub enum Power {
    HIGH,
    LOW,
}

// WHY DOESN'T THIS WORRRRRRRK
//fn digital_write(pin: u8, power: Power) {
//    let op_address: usize = match power {
//        Power::HIGH => 0x1C, // GPIO Output Set Register 0
//        Power::LOW => 0x28,  // GPIO Output Clear Register 0
//    };
//
//    unsafe {
//        core::ptr::write_volatile((BASE_ADDRESS + 0x28) as *mut u32, 1 << pin);
//    };
//}

fn digital_write(pin: u8, power: Power) {
    match power {
        Power::HIGH => unsafe {
            core::ptr::write_volatile(0x3F20_001C as *mut u32, 1 << pin);
        },
        Power::LOW => unsafe {
            core::ptr::write_volatile(0x3F20_0028 as *mut u32, 1 << pin);
        },
    }
}

//pub fn digital_read(pin: u8) -> usize {
//    return unsafe {
//        core::ptr::read_volatile( BASE_ADDRESS.wrapping_add((register * 4).into()) as *mut u32, sel_bits[index] << change_bit)
//    }
//}

/* to compile:
rm -r target/ .rp-files/kernel7.img &> /dev/null || echo "no binaries to delete" && cargo rustc -- -C link-arg=--script=./linker.ld && arm-none-eabi-objcopy -O binary target/armv7a-none-eabi/debug/farm-tings_-rust- ./.rp-files/kernel7.img
*/

/* to push:
sudo cp -v .rp-files/* /media/sd/ && sudo umount /media/sd/
*/ <- thinks the asterisk in the command is another block comment 🤦*/

/* esssential files:
 * https://github.com/raspberrypi/firmware/blob/master/boot/bootcode.bin
 * https://github.com/raspberrypi/firmware/blob/master/boot/fixup.dat
 * https://github.com/raspberrypi/firmware/blob/master/boot/start.elf
*/
