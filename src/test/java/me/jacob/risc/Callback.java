package me.jacob.risc;

import me.jacob.riscv.RV32_ICallback;
import me.jacob.riscv.RV32_IRAM;

public class Callback implements RV32_ICallback {
	public long handleControlLoad(long rsval) {
//		System.out.println("HCL FROM " + rsval);
		// TODO Auto-generated method stub
		if( rsval == 0x10000005 )
			return 0x60;
		
		return 0;
	}
	public int handleControlStore(long addy, long rs2) {
//		System.out.println("HCS to " + addy);
      if( addy == 0x10000000 ) //UART 8250 / 16550 Data Buffer
      {
    	  System.out.print((char)rs2);
//          tty.printf("%c", (char) val);
          System.out.flush();
      }
      return 0;
	}

	public void handleOtherCSRWrite(RV32_IRAM ram, long csrno, long writeval) {
//		System.out.println("CSR WRITE NO " + csrno);
		if (csrno == 0x136) {
		System.out.println(Integer.toString((int) writeval));
		System.out.flush();
		} else if (csrno == 0x137) {
			System.out.printf(Integer.toHexString((int) writeval));
			System.out.flush();
		} else if (csrno == 0x138) {
			//Print "string"
			long ptrstart = writeval - ArrayRAM.RAM_OFFSET;
			long ptrend = ptrstart;
			if (ptrstart >= ArrayRAM.RAM_AMOUNT)
				System.out.println("DEBUG PASSED INVALID PTR (" + Integer.toHexString((int) writeval) + ")");

			while (ptrend < ArrayRAM.RAM_AMOUNT) {
				if (ram.read((int) ptrend) == 0) break;
				System.out.print((char)ram.read((int) ptrend));
				ptrend++;
			}
			
			System.out.flush();
//			if (ptrend != ptrstart)
//				tty.write(image, (int) ptrstart, (int) ptrend);
			//String str = new String(Arrays.copyOfRange(image, (int) ptrstart, (int) ptrend));
		}
	}
	public long handleOtherCSRRead(RV32_IRAM ram, long csrno) {
//		System.out.println("HCSRR NO " + csrno);
		return 0;
	}

	public long handleException(long ir, long trap) {
		// TODO Auto-generated method stub
		System.out.println("EXCEPTION NO " + trap);
		System.out.println("Processor dump:");
		Main.emu.dumpState(Main.emu.getState());
		if(trap != 3) System.exit(-1);
		return trap;
	}
}
