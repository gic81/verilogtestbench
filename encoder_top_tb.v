`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   15:40:02 03/11/2015
// Design Name:   encoder_top
// Module Name:   /home/gic/encoder_virtex6_test/encoder_top/encoder_top_tb.v
// Project Name:  encoder_top
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: encoder_top
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module encoder_top_tb;

//*************************Parameter Declarations******************************

    parameter   TX_REFCLK_PERIOD   =   6.73;						//  148.5 MHz period
    parameter   RX_REFCLK_PERIOD   =   6.73;
    parameter   DCLK_PERIOD     =   37.037;//37.04;//20.0; 	//  27 MHz period
	 parameter   MIG_PERIOD     =   5;     						//  200 MHz period

	// Inputs
/*	reg Q0_CLK1_MGTREFCLK_PAD_N_IN;
	reg Q0_CLK1_MGTREFCLK_PAD_P_IN;
	reg Q1_CLK0_MGTREFCLK_PAD_N_IN;
	reg Q1_CLK0_MGTREFCLK_PAD_P_IN;
	reg DRP_CLK_IN;
	reg GTXTXRESET_IN;
	reg GTXRXRESET_IN;
	reg RXN_IN;
	reg RXP_IN;

	// Outputs
	wire TRACK_DATA_OUT;
	wire TXN_OUT;
	wire TXP_OUT;
	wire [9:0] y_check;
	wire [9:0] c_check;
	wire h_blank;
	wire v_blank;
	wire field;
	wire trs;
	wire xyz;
	wire [10:0] tx_line_num_check; */




//************************Internal Register Declarations***********************
//                                   INPUTS
//************************** Register Declarations ****************************        

    reg             tx_refclk_n_r;
    reg             rx_refclk_n_r;
    reg 	           mig_refclk_n; 
	 reg             drp_clk_r;
    reg             tx_usrclk_r;
    reg             rx_usrclk_r;    
    reg             gsr_r;
    reg             gts_r;
    reg             reset_i;
    reg             track_data_high_r;
    reg             track_data_low_r;

//********************************Wire Declarations**********************************
//                                   oUTPUTS
    //--------------------------------- Global Signals ------------------------------
   
	 wire            tx_refclk_p_r;
    wire            rx_refclk_p_r; 
	 wire            mig_refclk_p; 
    
    //-------------------------- Output Module Connections -------------------------
    wire            track_data_i;
    wire            rxn_in_i;
    wire            rxp_in_i;
    wire            txn_out_i;
    wire            txp_out_i;
	wire [9:0] y_tx_check;
	wire [9:0] c_tx_check;
	wire [9:0] y_rx_check;
	wire [9:0] c_rx_check;
	wire h_blank;
	wire v_blank;
	wire field;
	wire trs;
	wire xyz;
	wire [10:0] tx_line_num_check; 
	wire rx_trs;
	wire rx_xyz;
	wire [10:0] rx_line_num_check; 
	wire         	rx_clk;
	wire         	tx_clk;
	wire [19:0]		TRACK_DATA_OUT;
//*********************************Main Body of Code**********************************


    // ------------------------------- Tie offs -------------------------------- 
    
    assign  tied_to_ground_i     =    1'b0;
    
    // ------------------------- MGT Serial Connections ------------------------

    assign   rxn_in_i           =  txn_out_i;
    assign   rxp_in_i           =  txp_out_i;  

    //------------------------------ Global Signals ----------------------------
    
    //Simultate the global reset that occurs after configuration at the beginning
    //of the simulation. 
    assign glbl.GSR = gsr_r;
    assign glbl.GTS = gts_r;

    initial
        begin
            gts_r = 1'b0;        
            gsr_r = 1'b1;
            #(16*TX_REFCLK_PERIOD);
            gsr_r = 1'b0;
    end


    //---------- Generate Reference Clock input to UPPER MGTCLK ----------------
    
    initial begin
        tx_refclk_n_r = 1'b1;
    end

    always  
        #(TX_REFCLK_PERIOD/2) tx_refclk_n_r = !tx_refclk_n_r;

    assign tx_refclk_p_r = !tx_refclk_n_r;

    initial begin
        rx_refclk_n_r = 1'b1;
    end

    always  
        #(RX_REFCLK_PERIOD/2) rx_refclk_n_r = !rx_refclk_n_r;

    assign rx_refclk_p_r = !rx_refclk_n_r;
                 
    //------------------------ Generate DRP Clock ----------------------------
    
    initial begin
        drp_clk_r = 1'b1;
    end

    always  
        #(DCLK_PERIOD/2) drp_clk_r = !drp_clk_r;

    //---------- Generate Differencial Mig Reference Clock @ 200 MHz ----------------
    
    initial begin
        mig_refclk_n = 1'b1;
    end

    always  
        #(MIG_PERIOD/2) mig_refclk_n = !mig_refclk_n;

    assign mig_refclk_p = !mig_refclk_n;

    
    //--------------------------------- Resets ---------------------------------
    
    initial
    begin
        reset_i = 1'b1;
        #100 reset_i = 1'b0;
    end
    
    //----------------------------- Track Data ---------------------------------
    initial
    begin
        #12000000;
        $display("------- TEST COMPLETED -------");
        $stop;
    end
	// Instantiate the Unit Under Test (UUT)

	encoder_top uut (
		.Q0_CLK1_MGTREFCLK_PAD_N_IN				(rx_refclk_n_r), 
		.Q0_CLK1_MGTREFCLK_PAD_P_IN				(rx_refclk_p_r), 
		.Q1_CLK0_MGTREFCLK_PAD_N_IN				(tx_refclk_n_r), 
		.Q1_CLK0_MGTREFCLK_PAD_P_IN				(tx_refclk_n_r), 
		.DRP_CLK_IN										(drp_clk_r),
		.MIG_REF_CLK_N									(mig_refclk_n),
		.MIG_REF_CLK_P									(mig_refclk_p),
		.GTXTXRESET_IN									(reset_i), 
		.GTXRXRESET_IN									(reset_i), 
		.TRACK_DATA_OUT								(TRACK_DATA_OUT),//(track_data_i), 
		.RXN_IN											(rxn_in_i), 
		.RXP_IN											(rxp_in_i), 
		.TXN_OUT											(txn_out_i), 
		.TXP_OUT											(txp_out_i), 
		.y_tx_check										(y_tx_check), 
		.c_tx_check										(c_tx_check), 
		.y_rx_check 									(y_rx_check),
		.c_rx_check 									(c_rx_check),
		.h_blank											(h_blank), 
		.v_blank											(v_blank), 
		.field											(field), 
		.trs												(trs), 
		.xyz												(xyz), 
		.tx_line_num_check							(tx_line_num_check),
		.rx_clk											(rx_clk),
		.tx_clk											(tx_clk),	
		.rx_trs											(rx_trs), 
		.rx_xyz											(rx_xyz), 
		.rx_line_num_check							(rx_line_num_check)
		
	);

	
      
endmodule

