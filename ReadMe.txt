/************************************************************************
README
Code consists of Implementation for performing Static Timing Analysis incorporating Signal Integrity Effects 
For running the code use the following command

Make  -> object file cad2 is created

./cad2 ./NLDM/saed32_hvt_ss0p95v125c.lib ./NLDM/saed32_rvt_ss0p95v125c.lib ./NLDM/saed32_lvt_ss0p95v125c.lib  ./NLDM/saed32io_wb_ss0p95v125c_2p25v.lib ./c1908/c1908.vg ./c1908/c1908.spef.min ./c1908/c1908.spef.max 5 0.2 

where 
argv[1] : HVT NLDM file 
argv[2] : RVT NLDM file 
argv[3] : LVT NLDM file 
argv[4] : IO NLDM file 
argv[5] : Placed Netlist
argv[6] : SPEF min file  
argv[7] : SPEF max file  
argv[8] : clock 
argv[9] : input slew 

Code Authors: Kartheshwar, Akhil
************************************************************************/