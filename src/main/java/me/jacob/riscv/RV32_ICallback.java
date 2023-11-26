package me.jacob.riscv;

public interface RV32_ICallback {
	public long handleControlLoad(long rsval);
	public int handleControlStore(long addy, long rs2);
	
	public void handleOtherCSRWrite(RV32_IRAM ram, long csrno, long writeval);
	public long handleOtherCSRRead(RV32_IRAM ram, long csrno);
	
	public long handleException(long ir, long trap);
}
