package me.jacob.riscv;

public interface RV32_IRAM {
	public void write(int i, byte word);
	public byte read(int i);
	
	public long getSize();
	public long getOffset();
}
