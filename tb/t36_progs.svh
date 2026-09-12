// t36_progs.svh — instruction images for directed program testbenches.
// Each image lists the instruction address beside its encoded word.
task automatic load_prog_dual_a();
  int i;
  for (i = 0; i < 256; i++) imem[i] = 32'h0000_0013;
  imem[0] = 32'h00600313;  // pc 0x00
  imem[1] = 32'h00700393;  // pc 0x04
  imem[2] = 32'h027301b3;  // pc 0x08
  imem[3] = 32'h02318a63;  // pc 0x0c
  imem[4] = 32'h06019863;  // pc 0x10
  imem[5] = 32'h06300493;  // pc 0x14
  imem[6] = 32'h10902023;  // pc 0x18
  imem[16] = 32'h02a00293;  // pc 0x40
  imem[17] = 32'h10502223;  // pc 0x44
  imem[18] = 32'h0000006f;  // pc 0x48
  imem[32] = 32'h04d00513;  // pc 0x80
  imem[33] = 32'h10a02423;  // pc 0x84
endtask

task automatic load_prog_dual_b();
  int i;
  for (i = 0; i < 256; i++) imem[i] = 32'h0000_0013;
  imem[0] = 32'h00600313;  // pc 0x00
  imem[1] = 32'h00700393;  // pc 0x04
  imem[2] = 32'h027301b3;  // pc 0x08
  imem[3] = 32'h02319a63;  // pc 0x0c
  imem[4] = 32'h06318863;  // pc 0x10
  imem[5] = 32'h06300493;  // pc 0x14
  imem[6] = 32'h10902023;  // pc 0x18
  imem[32] = 32'h03700293;  // pc 0x80
  imem[33] = 32'h10502223;  // pc 0x84
  imem[34] = 32'h0000006f;  // pc 0x88
endtask

task automatic load_prog_dual_c();
  int i;
  for (i = 0; i < 256; i++) imem[i] = 32'h0000_0013;
  imem[0] = 32'h00600313;  // pc 0x00
  imem[1] = 32'h00700393;  // pc 0x04
  imem[2] = 32'h027301b3;  // pc 0x08
  imem[3] = 32'h06318a63;  // pc 0x0c
  imem[4] = 32'h02319063;  // pc 0x10
  imem[5] = 32'h06300493;  // pc 0x14
  imem[32] = 32'h04200293;  // pc 0x80
  imem[33] = 32'h10502223;  // pc 0x84
  imem[34] = 32'h0000006f;  // pc 0x88
endtask

task automatic load_prog_dual_d();
  int i;
  for (i = 0; i < 256; i++) imem[i] = 32'h0000_0013;
  imem[0] = 32'h00600313;  // pc 0x00
  imem[1] = 32'h00700393;  // pc 0x04
  imem[2] = 32'h027301b3;  // pc 0x08
  imem[3] = 32'h00319a63;  // pc 0x0c
  imem[4] = 32'h02018063;  // pc 0x10
  imem[5] = 32'h00b00493;  // pc 0x14
  imem[6] = 32'h10902023;  // pc 0x18
  imem[7] = 32'h00319463;  // pc 0x1c
  imem[8] = 32'h00319463;  // pc 0x20
  imem[9] = 32'h00319463;  // pc 0x24
  imem[10] = 32'h00319463;  // pc 0x28
  imem[11] = 32'h01600513;  // pc 0x2c
  imem[12] = 32'h10a02223;  // pc 0x30
  imem[13] = 32'h0000006f;  // pc 0x34
endtask

task automatic load_prog_stale_a();
  int i;
  for (i = 0; i < 256; i++) imem[i] = 32'h0000_0013;
  imem[0] = 32'h10000093;  // pc 0x00
  imem[1] = 32'h00600313;  // pc 0x04
  imem[2] = 32'h0040a103;  // pc 0x08
  imem[3] = 32'h00700393;  // pc 0x0c
  imem[4] = 32'h027361b3;  // pc 0x10  rem x3,x6,x7 (was mul: the release-cycle
                           // sweep needs the branch producer on the DIV
                           // family's 32-cycle FSM now that MUL is 2-stage;
                           // stale_b keeps its mul -- its div-by-zero holder
                           // tracks the broadcast shift and stays aligned)
  imem[5] = 32'h00318233;  // pc 0x14
  imem[6] = 32'h02318463;  // pc 0x18
  imem[7] = 32'h0000a483;  // pc 0x1c
  imem[8] = 32'h06300513;  // pc 0x20
  imem[16] = 32'h07700593;  // pc 0x40
  imem[17] = 32'h10b02623;  // pc 0x44
  imem[18] = 32'h0000006f;  // pc 0x48
endtask

task automatic load_prog_stale_b();
  int i;
  for (i = 0; i < 256; i++) imem[i] = 32'h0000_0013;
  imem[0] = 32'h00600313;  // pc 0x00
  imem[1] = 32'h00700393;  // pc 0x04
  imem[2] = 32'h027301b3;  // pc 0x08
  imem[3] = 32'h02318a63;  // pc 0x0c
  imem[4] = 32'h0201c4b3;  // pc 0x10
  imem[5] = 32'h06300513;  // pc 0x14
  imem[16] = 32'h05800293;  // pc 0x40
  imem[17] = 32'h10502823;  // pc 0x44
  imem[18] = 32'h0000006f;  // pc 0x48
endtask

task automatic load_prog_md_a();
  int i;
  for (i = 0; i < 256; i++) imem[i] = 32'h0000_0013;
  imem[0] = 32'h00600313;  // pc 0x00
  imem[1] = 32'h00700393;  // pc 0x04
  imem[2] = 32'h027301b3;  // pc 0x08
  imem[3] = 32'h02630a63;  // pc 0x0c
  imem[4] = 32'h06300493;  // pc 0x10
  imem[5] = 32'h10902023;  // pc 0x14
  imem[16] = 32'h10302a23;  // pc 0x40
  imem[17] = 32'h00500293;  // pc 0x44
  imem[18] = 32'h0000006f;  // pc 0x48
endtask

task automatic load_prog_md_b();
  int i;
  for (i = 0; i < 256; i++) imem[i] = 32'h0000_0013;
  imem[0] = 32'h00600313;  // pc 0x00
  imem[1] = 32'h00700393;  // pc 0x04
  imem[2] = 32'h027301b3;  // pc 0x08
  imem[3] = 32'h02738233;  // pc 0x0c
  imem[4] = 32'h10302c23;  // pc 0x10
  imem[5] = 32'h10402e23;  // pc 0x14
  imem[6] = 32'h0000006f;  // pc 0x18
endtask

task automatic load_prog_md_c();
  int i;
  for (i = 0; i < 256; i++) imem[i] = 32'h0000_0013;
  imem[0] = 32'h00600313;  // pc 0x00
  imem[1] = 32'h00700393;  // pc 0x04
  imem[2] = 32'h02000c63;  // pc 0x08
  imem[3] = 32'h020004b3;  // pc 0x0c
  imem[4] = 32'h06300513;  // pc 0x10
  imem[16] = 32'h0263c2b3;  // pc 0x40
  imem[17] = 32'h12502023;  // pc 0x44
  imem[18] = 32'h0000006f;  // pc 0x48
endtask

task automatic load_prog_iq_a();
  int i;
  for (i = 0; i < 256; i++) imem[i] = 32'h0000_0013;
  imem[0] = 32'h00600313;  // pc 0x00
  imem[1] = 32'h00700393;  // pc 0x04
  imem[2] = 32'h027301b3;  // pc 0x08
  imem[3] = 32'h0261c233;  // pc 0x0c
  imem[4] = 32'h0271c2b3;  // pc 0x10
  imem[5] = 32'h00318433;  // pc 0x14
  imem[6] = 32'h12402223;  // pc 0x18
  imem[7] = 32'h12502423;  // pc 0x1c
  imem[8] = 32'h12802623;  // pc 0x20
  imem[9] = 32'h0000006f;  // pc 0x24
endtask

task automatic load_prog_iq_b();
  int i;
  for (i = 0; i < 256; i++) imem[i] = 32'h0000_0013;
  imem[0] = 32'h00600313;  // pc 0x00
  imem[1] = 32'h00700393;  // pc 0x04
  imem[2] = 32'h0263c233;  // pc 0x08
  imem[3] = 32'h027342b3;  // pc 0x0c
  imem[4] = 32'h00730433;  // pc 0x10
  imem[5] = 32'h12402823;  // pc 0x14
  imem[6] = 32'h12502a23;  // pc 0x18
  imem[7] = 32'h12802c23;  // pc 0x1c
  imem[8] = 32'h0000006f;  // pc 0x20
endtask

task automatic load_prog_iq_c();
  int i;
  for (i = 0; i < 256; i++) imem[i] = 32'h0000_0013;
  imem[0] = 32'h10000093;  // pc 0x00
  imem[1] = 32'h00600313;  // pc 0x04
  imem[2] = 32'h3438d073;  // pc 0x08
  imem[3] = 32'h00700393;  // pc 0x0c
  imem[4] = 32'h0000a103;  // pc 0x10
  imem[6] = 32'h027301b3;  // pc 0x18
  imem[8] = 32'h12302e23;  // pc 0x20
  imem[9] = 32'h00618233;  // pc 0x24
  imem[10] = 32'h343192f3;  // pc 0x28
  imem[11] = 32'h00620433;  // pc 0x2c
  imem[12] = 32'h14502023;  // pc 0x30
  imem[13] = 32'h14802223;  // pc 0x34
  imem[14] = 32'h0000006f;  // pc 0x38
endtask

task automatic load_prog_ck_a();
  int i;
  for (i = 0; i < 256; i++) imem[i] = 32'h0000_0013;
  imem[0] = 32'h00600313;  // pc 0x00
  imem[1] = 32'h00700393;  // pc 0x04
  imem[2] = 32'h027301b3;  // pc 0x08
  imem[3] = 32'h02631a63;  // pc 0x0c
  imem[4] = 32'h02018863;  // pc 0x10
  imem[5] = 32'h00100493;  // pc 0x14
  imem[10] = 32'h00630c63;  // pc 0x28
  imem[11] = 32'h06300513;  // pc 0x2c
  imem[16] = 32'h02100593;  // pc 0x40
  imem[17] = 32'h14b02423;  // pc 0x44
  imem[18] = 32'h14902623;  // pc 0x48
  imem[19] = 32'h0000006f;  // pc 0x4c
endtask

task automatic load_prog_fl_a();
  int i;
  for (i = 0; i < 256; i++) imem[i] = 32'h0000_0013;
  imem[0] = 32'h00600313;  // pc 0x00
  imem[1] = 32'h00700393;  // pc 0x04
  imem[2] = 32'h027301b3;  // pc 0x08
  imem[3] = 32'h00100093;  // pc 0x0c
  imem[4] = 32'h00200113;  // pc 0x10
  imem[5] = 32'h00300213;  // pc 0x14
  imem[6] = 32'h00400293;  // pc 0x18
  imem[7] = 32'h00500413;  // pc 0x1c
  imem[8] = 32'h00600493;  // pc 0x20
  imem[9] = 32'h00700513;  // pc 0x24
  imem[10] = 32'h00800593;  // pc 0x28
  imem[11] = 32'h00900613;  // pc 0x2c
  imem[12] = 32'h00a00693;  // pc 0x30
  imem[13] = 32'h00b00713;  // pc 0x34
  imem[14] = 32'h00c00793;  // pc 0x38
  imem[15] = 32'h00d00813;  // pc 0x3c
  imem[16] = 32'h00e00893;  // pc 0x40
  imem[17] = 32'h00f00913;  // pc 0x44
  imem[18] = 32'h01000993;  // pc 0x48
  imem[19] = 32'h01100a13;  // pc 0x4c
  imem[20] = 32'h01200a93;  // pc 0x50
  imem[21] = 32'h01300b13;  // pc 0x54
  imem[22] = 32'h01400b93;  // pc 0x58
  imem[23] = 32'h01500c13;  // pc 0x5c
  imem[24] = 32'h01600c93;  // pc 0x60
  imem[25] = 32'h01700d13;  // pc 0x64
  imem[26] = 32'h01800d93;  // pc 0x68
  imem[27] = 32'h01900e13;  // pc 0x6c
  imem[28] = 32'h01a00e93;  // pc 0x70
  imem[29] = 32'h01b00f13;  // pc 0x74
  imem[30] = 32'h01c00f93;  // pc 0x78
  imem[31] = 32'h01d00093;  // pc 0x7c
  imem[32] = 32'h01e00113;  // pc 0x80
  imem[33] = 32'h01f00213;  // pc 0x84
  imem[34] = 32'h02000293;  // pc 0x88
  imem[35] = 32'h02100413;  // pc 0x8c
  imem[36] = 32'h14802823;  // pc 0x90
  imem[37] = 32'h0000006f;  // pc 0x94
endtask

task automatic load_prog_fl_b();
  int i;
  for (i = 0; i < 256; i++) imem[i] = 32'h0000_0013;
  imem[0] = 32'h00600313;  // pc 0x00
  imem[1] = 32'h00700393;  // pc 0x04
  imem[2] = 32'h027301b3;  // pc 0x08
  imem[3] = 32'h04318a63;  // pc 0x0c
  imem[4] = 32'h05000413;  // pc 0x10
  imem[5] = 32'h05100493;  // pc 0x14
  imem[6] = 32'h05200513;  // pc 0x18
  imem[7] = 32'h05300593;  // pc 0x1c
  imem[8] = 32'h05400613;  // pc 0x20
  imem[9] = 32'h05500693;  // pc 0x24
  imem[10] = 32'h05600713;  // pc 0x28
  imem[11] = 32'h05700793;  // pc 0x2c
  imem[12] = 32'h10002803;  // pc 0x30
  imem[13] = 32'h15002c23;  // pc 0x34
  imem[14] = 32'h06000893;  // pc 0x38
  imem[15] = 32'h06100913;  // pc 0x3c
  imem[16] = 32'h06200993;  // pc 0x40
  imem[17] = 32'h06300a13;  // pc 0x44
  imem[24] = 32'h02c00293;  // pc 0x60
  imem[25] = 32'h14502a23;  // pc 0x64
  imem[26] = 32'h0000006f;  // pc 0x68
endtask

task automatic load_prog_csr_a();
  int i;
  for (i = 0; i < 256; i++) imem[i] = 32'h0000_0013;
  imem[0] = 32'h00600313;  // pc 0x00
  imem[1] = 32'h34331073;  // pc 0x04
  imem[2] = 32'h00700393;  // pc 0x08
  imem[3] = 32'h027301b3;  // pc 0x0c
  imem[4] = 32'h02318863;  // pc 0x10
  imem[5] = 32'h3003a4f3;  // pc 0x14
  imem[6] = 32'h06300513;  // pc 0x18
  imem[16] = 32'h343025f3;  // pc 0x40
  imem[17] = 32'h343ad673;  // pc 0x44
  imem[18] = 32'h3432f6f3;  // pc 0x48
  imem[19] = 32'h14b02e23;  // pc 0x4c
  imem[20] = 32'h16d02023;  // pc 0x50
  imem[21] = 32'h0000006f;  // pc 0x54
endtask

task automatic load_prog_trap_a();
  int i;
  for (i = 0; i < 256; i++) imem[i] = 32'h0000_0013;
  imem[0] = 32'h08000093;  // pc 0x00
  imem[1] = 32'h30509073;  // pc 0x04
  imem[2] = 32'h17002103;  // pc 0x08
  imem[3] = 32'h00000073;  // pc 0x0c
  imem[4] = 32'h17402183;  // pc 0x10
  imem[5] = 32'h02318663;  // pc 0x14
  imem[6] = 32'h06300493;  // pc 0x18
  imem[16] = 32'h03700513;  // pc 0x40
  imem[17] = 32'h16a02423;  // pc 0x44
  imem[18] = 32'h0000006f;  // pc 0x48
  imem[32] = 32'h0ab00593;  // pc 0x80
  imem[33] = 32'h16b02223;  // pc 0x84
  imem[34] = 32'h34102673;  // pc 0x88
  imem[35] = 32'h34202773;  // pc 0x8c
  imem[36] = 32'h00460693;  // pc 0x90
  imem[37] = 32'h34169073;  // pc 0x94
  imem[38] = 32'h30200073;  // pc 0x98
endtask
