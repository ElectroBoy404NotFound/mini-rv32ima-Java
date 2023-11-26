package me.jacob.risc;


import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

import me.jacob.riscv.Rv32ima;

public class Main {
	public static Rv32ima emu;
    public static void main(String[] args) throws Exception {
        int show_help = 0;
        int time_divisor = 1;
        final boolean fixed_update = false;
        final boolean do_sleep = true;
        final boolean single_step = false;
        int dtb_ptr;
        long instct = -1;
        
        ArrayRAM ram = new ArrayRAM();
        
        String filepath = "linux";

        if (filepath != null) {
            byte[] prog = Files.readAllBytes(Path.of(filepath));
            int flen = prog.length;

            System.arraycopy(prog, 0, ram.ram, 0, flen);
        }

        byte[] dtb = Files.readAllBytes(Path.of("default64mbdtc.bin"));

        dtb_ptr = (int) (ram.getSize() - dtb.length - 192); // End of memory is saved for the state
        System.arraycopy(dtb, 0, ram.ram, dtb_ptr, dtb.length);
        System.out.println("ROM END: " + (dtb_ptr + dtb.length));

        Callback callback = new Callback();
        
        emu = new Rv32ima(callback, ram);
        /*
        TODO: this thing that idk is needed
        // The core lives at the end of RAM.
	    core = (struct MiniRV32IMAState *)(ram_image + RAMAmt - sizeof( struct MiniRV32IMAState ));
         */
        emu.getState().pc = ArrayRAM.RAM_OFFSET;
        emu.getState().regs[10] = 0; // hart ID
        emu.getState().regs[11] = dtb_ptr + ArrayRAM.RAM_OFFSET;
        emu.getState().extraflags |= 3; // Machine-mode.

        System.out.printf("PC: %d, dtb_ptr: %d\n", emu.getState().pc, dtb_ptr);

        //emu.dumpMem("imgdump.bin");

        // Image is loaded.
        long rt;
        long lastTime = fixed_update ? 0 : (getTimeMicroseconds() / time_divisor);
        int instrs_per_flip = single_step ? 1 : 1024;

        for (rt = 0; rt < instct + 1 || instct < 0; rt += instrs_per_flip) {
//            System.out.printf("    LAST_TIME: %d\n", lastTime);
            long this_ccount = emu.getState().cyclel;
            long elapsedUs;

            if (fixed_update)
                elapsedUs = this_ccount / time_divisor - lastTime;
            else
                elapsedUs = getTimeMicroseconds() / time_divisor - lastTime;
            lastTime += elapsedUs;

            if (single_step)
                emu.dumpState(emu.getState());

            int ret = (int) emu.rv32IMAStep(0, elapsedUs, instrs_per_flip);
//            System.out.println("RV STEP");
            switch (ret) {
                case 0 -> {}
                case 1 -> { this_ccount += instrs_per_flip;}
                case 3 -> instct = 0;
                case 0x7777 -> {System.out.printf( "RESTART@0x%08x%08x\n", emu.getState().cycleh, emu.getState().cyclel );} // TODO: Implement restart
                case 0x5555 -> {System.out.printf( "POWEROFF@0x%08x%08x\n", emu.getState().cycleh, emu.getState().cyclel ); return;} //syscon code for power-off
                default -> { System.out.println("Unknown failure " + ret); 
        		System.out.println("Processor dump:");
        		Main.emu.dumpState(Main.emu.getState());
        		System.exit(-1);}
            }
        }
        emu.dumpState(emu.getState());
    }
    
    static long getTimeMicroseconds() {
        //long ret = times.get(timeIndex++);
        //System.out.println(ret);
        //return ret;

        return System.nanoTime() / 1000;
        //return 0;
        //return (((int) System.currentTimeMillis() / 10) * 10) * 1000;
        //long ret = System.currentTimeMillis() * 1000;
        //times.add(ret);
        //if (times.size() >= 200000) {
        //    try {
        //        Files.writeString(Path.of(timepath), times.stream().map(Object::toString).collect(Collectors.joining("\n")));
        //    } catch (Exception e) {
        //        throw new RuntimeException(e);
        //    }
        //    System.exit(0);
        //}
        //return ret;
    }
}
