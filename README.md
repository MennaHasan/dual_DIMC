# dual_DIMC
Implementation of two independently controlled dimc macros with shared FIFO
data paths and double buffering. `clk` and `rst_n` are shared. Every other dimc
control input is provided independently using `_m0` and `_m1` ports. `sel` only
chooses which macro output is forwarded to the output FIFO; it does not gate
either macro's input controls.

STEPS:
1. modules load
module load bender/0.31.0   
module load questasim

2. comment or uncomment test defines to select comiled tests

3. compile modules and testbenches
make hw-clean
make hw-all
make sim-single
make sim-dual
make sim-cleopatra
make sim-double-buffering


4. To use GUI 
make sim-single GUI=1
make sim-dual GUI=1
make sim-cleopatra GUI=1

5. Adding signals innside Questasim 
A. for sim-dual
restart -f
env tb_dimc_dual
add wave clk COMPE RCSN READYN PSOUT SOUT RES_OUT out_data out_empty out_pop

B. for sim-cleopatra
restart -f
env tb_cleopatra
add wave clk COMPE acc_clear_i acc_o
add wave sim:/tb_cleopatra/i_dut/READYN
add wave sim:/tb_cleopatra/i_dut/out_data

6. run simulation in Questasim
run -all
