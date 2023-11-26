package me.jacob.risc;

import me.jacob.riscv.RV32_IRAM;

public class ArrayRAM implements RV32_IRAM {
	public static final long RAM_OFFSET = 0x80000000L;
	public static final int  RAM_AMOUNT = 64 * 1024 * 1024;
	
	public byte[] ram;
	
	public ArrayRAM() {
		ram = new byte[RAM_AMOUNT];
	}
	
	public void write(int i, byte word) {
//		System.out.println("RAM WRITE TO " + i);
		ram[i] = word;
	}
	public byte read(int i) {
//		System.out.println("RAM READ TO " + i + " AND RETURN " + ram[i]);
		return ram[i];
	}

	public long getSize() {
		return RAM_AMOUNT;
	}
	public long getOffset() {
		return RAM_OFFSET;
	}
}
