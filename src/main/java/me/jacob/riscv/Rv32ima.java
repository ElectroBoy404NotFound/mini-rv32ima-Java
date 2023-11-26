package me.jacob.riscv;

public class Rv32ima {
//	public static final TTY tty = new TTY();
	
//	private long RAMOffest = 0x80000000L;
//	private int RAMAmt = 64*1024*1024;
	
	private RV32_ICallback callback;
	private RV32_IRAM ram;
	
	private State state = new State();
	private int fail_on_all_faults = 0;

	public Rv32ima(RV32_ICallback callback, RV32_IRAM ram) {
		this.callback = callback;
		this.ram = ram;
	}

	private void rv32_store(long ofs, long val, int words) { //TODO: This might not be little/big indian
		for (int i = 0; i < words; i++) {
//			ram.write((int) (ofs + i), word);
			ram.write((int) ofs + i, (byte) ((val >> i*8) & 0xff));
		}
	}

	private long rv32_load(long ofs, int words) { //TODO: This might not be little/big indian
		long ret = 0;

		for (int i = 0; i < words; i++) {
//			ret |= Byte.toUnsignedLong((byte) (ram.read((int) ofs + i) << (i * 8)));
//			ret <<= 8;
			byte word = ram.read((int) (ofs + i));
			ret |= Byte.toUnsignedLong(word) << (i*8);
		}
		return ret;
	}

	private void rv32_store4(long ofs, long val) {rv32_store(ofs, val, 4);}
	private void rv32_store2(long ofs, long val) {rv32_store(ofs, val, 2);}
	private void rv32_store1(long ofs, long val) {rv32_store(ofs, val, 1);}
	private long rv32_load4(long ofs) {return rv32_load(ofs, 4);}
	private int rv32_load2(long ofs) {return (int) rv32_load(ofs, 2);}
	private short rv32_load1(long ofs) {return (short) rv32_load(ofs, 1);}

//	private void regDump() {
//		long[] regs = state.regs;
//		System.out.printf( "Z:%08x ra:%08x sp:%08x gp:%08x tp:%08x t0:%08x t1:%08x t2:%08x s0:%08x s1:%08x a0:%08x a1:%08x a2:%08x a3:%08x a4:%08x a5:%08x ",
//				regs[0], regs[1], regs[2], regs[3], regs[4], regs[5], regs[6], regs[7],
//				regs[8], regs[9], regs[10], regs[11], regs[12], regs[13], regs[14], regs[15] );
//		System.out.printf( "a6:%08x a7:%08x s2:%08x s3:%08x s4:%08x s5:%08x s6:%08x s7:%08x s8:%08x s9:%08x s10:%08x s11:%08x t3:%08x t4:%08x t5:%08x t6:%08x\n",
//				regs[16], regs[17], regs[18], regs[19], regs[20], regs[21], regs[22], regs[23],
//				regs[24], regs[25], regs[26], regs[27], regs[28], regs[29], regs[30], regs[31] );
//	}
//
//	private String extra() {
//		if (useRegistersInDebug) {
//			long[] regs = state.regs;
//			return String.format("; REG: %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
//					regs[0], regs[1], regs[2], regs[3], regs[4], regs[5], regs[6], regs[7], regs[8], regs[9], regs[10], regs[11], regs[12], regs[13], regs[14], regs[15], regs[16], regs[17], regs[18], regs[19], regs[20], regs[21], regs[22], regs[23], regs[24], regs[25], regs[26], regs[27], regs[28], regs[29], regs[30], regs[31]);
//		} else if (useIRInDebug)
//			return String.format("; %08x", ir);
//		return "";
//	}

//	static {
//		try {
//			//times = Files.readAllLines(Path.of(timepath)).stream().map(Long::parseLong).toList();
//		} catch (Exception e) {
//			throw new RuntimeException(e);
//		}
//	}

//	static long totalExecuted = 0;
//	final boolean debug = false;
//	final boolean useRegistersInDebug = false;
//	final boolean useIRInDebug = false;
//	final boolean showTraps = true;
//	final long dumpStart = 2962953;//2962953;
//	final long debugStart = 92962953;//2962953;
//	final long debugLength = 199000;

	private long ir;
	private long rval;

	public long rv32IMAStep(long vProcAddress, long elapsedUs, int count) {
		long new_timer = getState().timerl + elapsedUs;
		if (new_timer < getState().timerl) getState().timerh++;
		getState().timerl = new_timer;

		if ((getState().timerh > getState().timermatchh || (getState().timerh == getState().timermatchh && getState().timerl > getState().timermatchl)) && (getState().timermatchh > 0 || getState().timermatchl > 0)) {
			getState().extraflags &= ~4;
			getState().mip |= 1<<7;
		} else
			getState().mip &= ~(1<<7);
		if ((getState().extraflags & 4) > 0)
			return 1;

		//if (totalExecuted >= debugStart && debug) System.out.printf("   ELAPSED: %d, %d\n", elapsedUs, state.timerl);

		int icount;
		for (icount = 0; icount < count; icount++) {
//			System.out.println("CYCLE START");
			ir = 0;
			long trap = 0;
			rval = 0;

			getState().cyclel++;
			if (getState().cyclel == 0) getState().cycleh++;

			long pc = getState().pc;
			long ofs_pc = pc - ram.getOffset();

			if (ofs_pc >= ram.getSize())
				trap = 1 + 1;
			else if ((pc & 3) > 0)
				trap = 1 + 0;
			else {
				String.valueOf(getState().timerl);

				ir = rv32_load4(ofs_pc);
				long rdid = (ir >> 7) & 0x1f;
//				totalExecuted++;
				switch ((int) (ir & 0x7f)) {
					case 0b0110111 -> {
						rval = (ir & 0xfffff000L); // LUI
						 }
					case 0b0010111 -> {
						rval = (pc + (ir & 0xfffff000L)) & 0xFFFFFFFFL; // AUIPC
						 }
					case 0b1101111 -> { // JAL
//						System.out.println("IR HAS " + ir);
						int reladdy = (int) (((ir & 0x80000000L)>>11) | ((ir & 0x7fe00000)>>20) | ((ir & 0x00100000)>>9) | ((ir&0x000ff000)));
						if((reladdy & 0x00100000) > 0) reladdy |= 0xffe00000L; // Sign extension.

						rval = pc + 4;
						pc = pc + reladdy - 4;
//						System.out.println("INSTRUCTION JAL WITH ADDR " + reladdy);
					}
					case 0b1100111 -> { // JALR
						long imm = ir >> 20;
						int imm_se = (int) (imm | (((imm & 0x800) > 0) ? 0xfffff000 : 0));
						rval = pc + 4;

						//if (totalExecuted >= debugStart && debug) System.out.printf("%08x: JALR %d, %d%s\n", pc, imm, imm_se, extra());

						pc = ( (getState().regs[(int) ((ir >> 15) & 0x1f)] + imm_se) & ~1) - 4;
						 }
					case 0b1100011 -> { // BRANCH
						long immm4 = ((ir & 0xf00) >> 7) | ((ir & 0x7e000000) >> 20) | ((ir & 0x80) << 4) | ((ir >> 31) << 12);
						if ((immm4 & 0x1000) > 0) immm4 |= 0xffffe000L;

						int rs1 = (int) getState().regs[(int) (ir >> 15) & 0x1f];
						int rs2 = (int) getState().regs[(int) (ir >> 20) & 0x1f];

						immm4 = (((pc + immm4) & 0xFFFFFFFFL) - 4) & 0xFFFFFFFFL;
						rdid = 0;
						switch ((int) ((ir >> 12) & 0x7)) {
							case 0b000 -> {if(rs1 == rs2) pc = immm4;}
							case 0b001 -> {if(rs1 != rs2) pc = immm4;}
							case 0b100 -> {if(rs1 < rs2) pc = immm4;}
							case 0b101 -> {if(rs1 >= rs2) pc = immm4;}
							case 0b110 -> {if(Integer.toUnsignedLong(rs1) < Integer.toUnsignedLong(rs2)) pc = immm4;}
							case 0b111 -> {if(Integer.toUnsignedLong(rs1) >= Integer.toUnsignedLong(rs2)) pc = immm4;}
							default -> { trap = (2+1); System.out.println("BRANCH OPCODE FAIL"); }
						}
					}
					case 0b0000011 -> { // Load
						long rs1 = getState().regs[(int) ((ir >> 15) & 0x1f)];
						long imm = (ir >> 20) & 0xFFFFFFFFL;
						int imm_se = (int) (imm | (((imm & 0x800) > 0) ? 0xfffff000 : 0));
						long rsval = (rs1 + Integer.toUnsignedLong(imm_se)) & 0xFFFFFFFFL;

						rsval -= ram.getOffset();
						rsval &= 0xFFFFFFFFL;

						if (rsval >= ram.getSize() - 3  ) {

							rsval -= ram.getOffset();
							rsval &= 0xFFFFFFFFL;
							if (rsval >= 0x10000000L && rsval < 0x12000000L) {
								if (rsval == 0x1100bffcL)
									rval = getState().timerh;
								else if (rsval == 0x1100bff8L)
									rval = getState().timerl;
								else
									rval = callback.handleControlLoad(rsval);
							} else {
								System.out.println("LOAD ACCESS FAULT");
								trap = (5 + 1);
								rval = rsval;
							}
						} else {
							//if (totalExecuted >= debugStart && debug) System.out.println("Bottom " + ((ir >> 12) & 0x7) + " " + rsval + " " + rv32_load4(rsval));
							switch ((int) ((ir >> 12) & 0x7)) {
								case 0b000 -> rval = (byte) rv32_load1(rsval); // Possibly cast to byte before setting rval
								case 0b001 -> rval = (short) rv32_load2(rsval); // Possibly cast to short before setting rval
								case 0b010 -> rval = rv32_load4(rsval);
								case 0b100 -> rval = rv32_load1(rsval);
								case 0b101 -> rval = rv32_load2(rsval);
								default -> { System.out.println("LOAD OPF"); trap = (2 + 1); }
							}
						}
					}
					case 0b0100011 -> { // Store
						long rs1 = getState().regs[(int) ((ir >> 15) & 0x1f)];
						long rs2 = getState().regs[(int) ((ir >> 20) & 0x1f)];
						long addy = (((ir >> 7) & 0x1f) | ((ir & 0xfe000000L) >> 20) & 0xFFFFFFFFL);

						if ((addy & 0x800) > 0) addy |= 0xfffff000L;
						addy += (rs1 - ram.getOffset()) & 0xFFFFFFFFL;
						addy &= 0xFFFFFFFFL;
						rdid = 0;

						if (addy >= ram.getSize() - 3) {
							addy -= ram.getOffset();
							addy &= 0xFFFFFFFFL;
							if (addy >= 0x10000000 && addy < 0x12000000) {

								// Should only be stuff like SYSCON, 8250, CLNT
								if (addy == 0x11004004) { // CLNT
									getState().timermatchh = rs2;
								} else if (addy == 0x11004000) { // CLNT
									getState().timermatchl = rs2;
								} else if (addy == 0x11100000) { // SYSCON (reboot, poweroff, etc.)
									getState().pc = pc + 4;
									return rs2;
								} else {
									if(callback.handleControlStore( addy, rs2 ) > 0) return rs2;
								}
							} else {
								System.out.println("STORE ACCESS FAULT TO " + addy);
								trap = (7 + 1); // Store access fault.
								rval = addy + ram.getOffset();
							}
						} else {
							switch ((int) ((ir >> 12) & 0x7)) {
								//SB, SH, SW
								case 0b000 -> rv32_store1(addy, rs2);
								case 0b001 -> rv32_store2(addy, rs2);
								case 0b010 -> rv32_store4(addy, rs2);
								default -> { System.out.println("STORE OPF"); trap = (2 + 1); }
							}
						}
					}
					case 0b0010011, 0b0110011 -> { // Op / Op-immediate
						long imm = ir >> 20;
						imm = imm | (((imm & 0x800) > 0) ? 0xfffff000 : 0);
						imm = imm & 0xFFFFFFFFL;

						long rs1 = getState().regs[(int) ((ir >> 15) & 0x1f)];
						boolean is_reg = (ir & 0b100000) > 0;
						long rs2 = is_reg ? getState().regs[(int) (imm & 0x1f)] : imm;

						if (is_reg && (ir & 0x02000000) > 0) {
						   // if (totalExecuted >= debugStart && debug) System.out.println("topHalf " + ((ir >> 12) & 7));
							switch ((int) ((ir >> 12) & 7)) { // 0x02000000 = RV32M
								case 0b000 -> rval = rs1 * rs2; // MUL
								case 0b001 -> rval = ((long) ((int) rs1) * (long) ((int) rs2)) >> 32; // MULH
								case 0b010 -> rval = ((long)((int)rs1) * rs2) >> 32; // MULHSU
								case 0b011 -> rval = (rs1 * rs2) >> 32; // MULHU
								case 0b100 -> {if (rs2 == 0) rval =  -1; else rval =  ((int) rs1 / (int) rs2);} // DIV
								case 0b101 -> {if (rs2 == 0) rval =  0xffffffffL; else rval =  rs1 / rs2;} // DIVU
								case 0b110 -> {if (rs2 == 0) rval =  rs1; else rval =  ((int) rs1) % ((int) rs2);} // REM
								case 0b111 -> {if (rs2 == 0) rval =  rs1; else rval =  rs1 % rs2;} // REMU
							};
						} else { // These could be either op-immediate or op commands.  Be careful.
							//if (totalExecuted >= debugStart && debug) System.out.println("bottomHalf " + ((ir >> 12) & 7));
							switch ((int) ((ir >> 12) & 7)) {
								case 0b000 -> rval = (is_reg && (ir & 0x40000000) > 0) ? (rs1 - rs2) : (rs1 + rs2);
								case 0b001 -> rval = rs1 << (rs2 % 32);
								case 0b010 -> rval = ((int) rs1) < ((int) rs2) ? 1 : 0;
								case 0b011 -> rval = rs1 < rs2 ? 1 : 0;
								case 0b100 -> rval = rs1 ^ rs2;
								case 0b101 -> rval = ((ir & 0x40000000) > 0) ? (((int) rs1) >> (rs2 % 32)) : (rs1 >> (rs2 % 32));
								case 0b110 -> rval = rs1 | rs2;
								case 0b111 -> rval = rs1 & rs2;
							};
						}
					}
					case 0b0001111 -> {rdid = 0;} // fencetype = (ir >> 12) & 0b111; We ignore fences in this impl.
					case 0b1110011 -> { // Zifencei+Zicsr
						long csrno = ir >> 20;
						int microop = (int) ((ir >> 12) & 0b111);


						if ((microop & 3) > 0) { // It's a Zicsr function.
							int rs1imm = (int) ((ir >> 15) & 0x1f);
							long rs1 = getState().regs[rs1imm];
							long writeval = rs1;

							// https://raw.githubusercontent.com/riscv/virtual-memory/main/specs/663-Svpbmt.pdf
							// Generally, support for Zicsr
							//if (totalExecuted >= debugStart && debug) System.out.println(state.mie + " " + state.mip + " " + state.mepc + " " + state.mstatus + " " + state.mtval);
							switch ((int) csrno) {
								case 0x340 -> rval = getState().mscratch;
								case 0x305 -> rval = getState().mtvec;
								case 0x304 -> rval = getState().mie;
								case 0xC00 -> rval = getState().cyclel;
								case 0x344 -> rval = getState().mip;
								case 0x341 -> rval = getState().mepc;
								case 0x300 -> rval = getState().mstatus;
								case 0x342 -> rval = getState().mcause;
								case 0x343 -> rval = getState().mtval;
								case 0xf11 -> rval = 0xff0ff0ffL; //mvendorid
								case 0x301 -> rval = 0x40401101L; //misa (XLEN=32, IMA+X)
								default -> rval = callback.handleOtherCSRRead(ram, csrno);
							};

							writeval = switch (microop) {
								case 0b001 -> rs1;
								case 0b010 -> rval | rs1;
								case 0b011 -> rval & ~rs1;
								case 0b101 -> rs1imm;
								case 0b110 -> rval | rs1imm;
								case 0b111 -> rval & ~rs1imm;
								default -> rval;
							};

							switch ((int) csrno) {
								case 0x340 -> getState().mscratch = writeval;
								case 0x305 -> getState().mtvec = writeval;
								case 0x304 -> getState().mie = writeval;
								case 0x344 -> getState().mip = writeval;
								case 0x341 -> getState().mepc = writeval;
								case 0x300 -> getState().mstatus = writeval;
								case 0x342 -> getState().mcause = writeval;
								case 0x343 -> getState().mtval = writeval;
								default -> callback.handleOtherCSRWrite(ram, csrno, writeval);
							}
						   // if (totalExecuted >= debugStart && debug) System.out.println(state.mie + " " + state.mip + " " + state.mepc + " " + state.mstatus + " " + state.mcause + " " + state.mtval);
						} else if (microop == 0b000) { // "SYSTEM"

							rdid = 0;
							if (csrno == 0x105) { // WFI (Wait for interrupts
								getState().mstatus |= 8; // Enable interrupts
								getState().extraflags |= 4; // Inform environment we want to go to sleep.
								getState().pc = pc + 4;

								return 1;
							} else if ((csrno & 0xff) == 0x02) { // MRET

								//https://raw.githubusercontent.com/riscv/virtual-memory/main/specs/663-Svpbmt.pdf
								//Table 7.6. MRET then in mstatus/mstatush sets MPV=0, MPP=0, MIE=MPIE, and MPIE=1. La
								// Should also update mstatus to reflect correct mode.
								long startmstatus = getState().mstatus;
								long startextraflags = getState().extraflags;
								getState().mstatus = (((startmstatus & 0x80) >> 4) | ((startextraflags & 3) << 11) | 0x80) & 0xFFFFFFFFL;
								getState().extraflags = ((startextraflags & ~3) | ((startmstatus >> 11) & 3)) & 0xFFFFFFFFL;

								pc = getState().mepc - 4;
							} else {

								switch ((int) csrno) {
									case 0 -> {
										//System.out.printf("%08x: ECALL fault", pc);
										trap = ((getState().extraflags & 3) > 0 ? (11 + 1) : (8 + 1)); // ECALL; 8 = "Environment call from U-mode"; 11 = "Environment call from M-mode"
									}
									case 1 -> {
										trap = (3 + 1); // EBREAK 3 = "Breakpoint"
										//System.err.println("Breakpoint hit!");
									}
									default -> { System.out.println("SYSTEM OPF"); trap = (2 + 1); }
								}
							}
						} else {
							System.out.println("Z+Z OPF"); trap = (2 + 1); 
							//if (totalExecuted >= min && desc) System.out.printf("%08x: ZIFENCEI ERROR %d%s\n", pc, trap, extra());
						}
					}
					case 0b0101111 -> { // RV32A
						long rs1 = getState().regs[(int) ((ir >> 15) & 0x1f)];
						long rs2 = getState().regs[(int) ((ir >> 20) & 0x1f)];
						long irmid = (ir >> 27) & 0x1f;

						rs1 -= ram.getOffset();

						// We don't implement load/store from UART or CLNT with RV32A here.
						if (rs1 >= ram.getSize() - 3) {
							System.out.println("STORE/AMO ACCESS FAULT TO " + rs1);
							trap = (7 + 1); // Store/AMO access fault
							rval = rs1 + ram.getOffset();
						} else {

							rval = rv32_load4(rs1);

							// Referenced a bit of https://github.com/franzflasch/riscv_em/blob/master/src/core/core.c
							boolean dowrite = true;
							switch ((int) irmid) {
								case 0b00010 -> dowrite = false; // LR.W
								case 0b00011 -> rval = 0; // SC.W (Lie and always say it's good)
								case 0b00001 -> {} // AMOSWAP.W
								case 0b00000 -> rs2 += rval; // AMOADD.W
								case 0b00100 -> rs2 ^= rval; // AMOXOR.W
								case 0b01100 -> rs2 &= rval; // AMOAND.W
								case 0b01000 -> rs2 |= rval; // AMOOR.W
								case 0b10000 -> rs2 = (((int) rs2) < ((int) rval)) ? rs2 : rval; // AMOMIN.W
								case 0b10100 -> rs2 = (((int) rs2) > ((int) rval)) ? rs2 : rval; // AMOMAX.W
								case 0b11000 -> rs2 = Math.min(rs2, rval); // AMOMINU.W
								case 0b11100 -> rs2 = Math.max(rs2, rval); // AMOMAX.W
								default -> {System.out.println("RV32A OPF");  trap = (2 + 1); dowrite = false;} // Not supported.
							}
							if (dowrite) rv32_store4(rs1, rs2);
						}
					}
					default -> {
						System.out.println("GENERAL OPF"); trap = (2 + 1); 
						 }
				}
				if (trap == 0) {
					if (rdid > 0) {
						//System.out.println(state.regs[1] + " " + rval + " " + rdid);
						getState().regs[(int) rdid] = (rval) & 0xFFFFFFFFL;
					} else if (((getState().mip & (1 << 7)) > 0) && ((getState().mie & (1 << 7) /*mtie*/) > 0) && ((getState().mstatus & 0x8 /*mie*/) > 0))
						trap = 0x80000007L; // Timer interrupt.
				}
//				System.out.println("CYCLE END");

			}
			
			if( trap > 0 ) {if(fail_on_all_faults > 0) {System.out.println("FAULT");} else trap = callback.handleException(ir, trap); System.out.println("TRAP");}

			// handle traps and interrupts.
			if (trap > 0) {
				if ((trap & 0x80000000L) > 0) {
					getState().mcause = trap;
					getState().mtval = 0;
					pc += 4;
				} else {
					getState().mcause = trap - 1;
					getState().mtval = (trap > 5 && trap <= 8) ? rval : pc;
				}
				getState().mepc = pc; //TRICKY: The kernel advances mepc automatically.
				//state.mstatus & 8 = MIE, & 0x80 = MPIE
				// On an interrupt, the system moves current MIE into MPIE
				getState().mstatus = ((getState().mstatus & 0x08) << 4) | (( getState().extraflags & 3 ) << 11);
				pc = getState().mtvec - 4;

				if ((trap & 0x80000000L) < 1)
					getState().extraflags |= 3;
				System.out.println("TRAP");
			}

			getState().pc = pc + 4;
		}
		return 0;
	}
	
	public void dumpState(State core) {
		long pc = core.pc;
		long pc_offset = pc - ram.getOffset();
		long ir = 0;

		System.out.printf( "PC: %08x ", pc );
		if( pc_offset >= 0 && pc_offset < ram.getSize() - 3 )
		{
//			ir = ram_image[(int) pc_offset];
			ir = ram.read((int) pc_offset);
			System.out.printf( "[0x%08x] ", ir );
		}
		else
			System.out.print( "[xxxxxxxxxx] " );
		long[] regs = core.regs;
		System.out.printf( "Z:%08x ra:%08x sp:%08x gp:%08x tp:%08x t0:%08x t1:%08x t2:%08x s0:%08x s1:%08x a0:%08x a1:%08x a2:%08x a3:%08x a4:%08x a5:%08x\n",
				regs[0], regs[1], regs[2], regs[3], regs[4], regs[5], regs[6], regs[7],
				regs[8], regs[9], regs[10], regs[11], regs[12], regs[13], regs[14], regs[15] );
		System.out.printf( "a6:%08x a7:%08x s2:%08x s3:%08x s4:%08x s5:%08x s6:%08x s7:%08x s8:%08x s9:%08x s10:%08x s11:%08x t3:%08x t4:%08x t5:%08x t6:%08x\n",
				regs[16], regs[17], regs[18], regs[19], regs[20], regs[21], regs[22], regs[23],
				regs[24], regs[25], regs[26], regs[27], regs[28], regs[29], regs[30], regs[31] );
	}
	
	public State getState() {
		return state;
	}

	public void setState(State state) {
		this.state = state;
	}

	public class State {
		public long[] regs = new long[32];

		public long pc;
		public long mstatus;
		public long cyclel;
		public long cycleh;

		public long timerl;
		public long timerh;
		public long timermatchl;
		public long timermatchh;

		public long mscratch;
		public long mtvec;
		public long mie;
		public long mip;

		public long mepc;
		public long mtval;
		public long mcause;

		// Note: only a few bits are used.  (Machine = 3, User = 0)
		// Bits 0..1 = privilege.
		// Bit 2 = WFI (Wait for interrupt)
		public long extraflags;
	}

}
