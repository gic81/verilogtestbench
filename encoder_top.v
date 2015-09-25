`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Gatv
// Engineer: Giuseppe Conti 
// 
// Create Date:    15:37:34 03/05/2015 
// Design Name: 
// Module Name:    encoder_top 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description:  Test Bench module able to connect the signal generator to the Triple rate sdi and GTX Traceiver 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
module encoder_top(
    
		input wire  Q0_CLK1_MGTREFCLK_PAD_N_IN,		// rx reference clk n
		input wire  Q0_CLK1_MGTREFCLK_PAD_P_IN,		// rx reference clk p 
		input wire  Q1_CLK0_MGTREFCLK_PAD_N_IN,		// tx reference clk n 
		input wire  Q1_CLK0_MGTREFCLK_PAD_P_IN,		// tx reference clk p
		input wire  DRP_CLK_IN,
		input wire 	MIG_REF_CLK_N,
		input wire	MIG_REF_CLK_P,
		
		input wire  GTXTXRESET_IN,
		input wire  GTXRXRESET_IN,
		output wire [19:0]  TRACK_DATA_OUT,
//		input  wire         RXN_IN,
//		input  wire         RXP_IN,
		output wire         RXN_IN,
		output wire         RXP_IN,
		output wire         TXN_OUT,
		output wire         TXP_OUT,
		output wire	[9:0]		y_tx_check,          // luma output channel
		output wire	[9:0]  	c_tx_check,          // chroma output channel
		output wire	[9:0]		y_rx_check,          // luma input channel
		output wire	[9:0]  	c_rx_check,          // chroma input channel
		output wire        	h_blank,    // asserted during horizontal blanking period
		output wire        	v_blank,    // asserted 
		output wire        	field,      // indicates current field
		output wire        	trs,        // asserted during 4 words of TRS symbol,
		output wire        	xyz,        // asserted during TRS XYZ word
		output wire	[10:0] 	tx_line_num_check,    // current vertical line number
		output wire         	rx_clk,
		output wire         	tx_clk,
		output wire 			rx_trs, 
		output wire  			rx_xyz, 
		output wire	[10:0] 	rx_line_num_check
    );


// RX2 signals
wire        rx2_recclk;                             // MGT RXRECCLK
wire        rx2_clr_errs;                           // User control to clear RX CRC & EDH error counts
wire        rx2_usrclk;                             // Global RXUSRCLK2
wire [1:0]  rx2_mode;                               // RX SDI mode: 00=HD, 01=SD, 10=3G
wire        rx2_mode_HD;                            // 1 = RX SDI mode = HD
wire        rx2_mode_SD;                            // 1 = RX SDI mode = SD
wire        rx2_mode_3G;                            // 1 = RX SDI mode = 3G
wire        rx2_mode_locked;                        // 1 = RX SDI mode detection locked
wire        rx2_m;                                  // 1 = input bit rate is 1/1.001
wire        rx2_locked;                             // 1 = RX format detector locked
wire [3:0]  rx2_t_family;                           // Incoming signal transport format family
wire [3:0]  rx2_t_rate;                             // incoming signal transport frame rate
wire        rx2_t_scan;                             // incoming signal transport scan mode
wire        rx2_level_b;                            // 1 = input signal is 3G-SDI level B
wire        rx2_ce;                                 // RX SD clock enable
wire [10:0] rx2_ln_a;                               // RX line number link A
wire [10:0] rx2_ln_b;                               // RX line number link B
wire [31:0] rx2_a_vpid;                             // SMPTE 352 video payload ID data link A data stream
wire        rx2_a_vpid_valid;                       // 1 = rx2_a_vpid valid
wire [31:0] rx2_b_vpid;                             // SMPTE 352 video payload ID data link B data stream
wire        rx2_b_vpid_valid;                       // 1 = rx2_b_vpid_valid
wire        rx2_crc_err_a;                          // CRC error detected, link A data stream
wire        rx2_crc_err_b;                          // CRC error detected, link B data stream (3G-B only)
wire        rx2_eav;                                // RX EAV
wire        rx2_sav;                                // RX SAV
wire			rx2_trs;											 // RX TRS added by gic
wire        rx2_dout_rdy_3G;                        // RX 3G-B clock enable
wire [9:0]  rx2_ds1b;                               // Link B data stream 1 (3G-B only)
wire [9:0]  rx2_ds2b;                               // Link B data stream 2 (3G-B only)
wire [15:0] rx2_edh_errcnt;                         // SD-SDI EDH error count
wire        rx2_mode_SD_sync;                       // rx2_mode_SD synced to tx2_usrclk

// TX2 signals
wire        tx2_outclk;                             // TXOUTCLK from TX2 MGT
wire        tx2_usrclk;                             // Driven by RXRECCLK for HD/3G and TXOUTCLK for SD(* equivalent_register_removal = "no" *)
wire        tx2_din_rdy;                            // TX clock enable used in 3G-SDI level B mode
wire        rx2_level_b_sync;                       // rx2_level_b synced to tx2_usrclk
reg  [1:0]  tx2_mode = 2'b00;                       // TX SDI mode: 00=HD, 01=SD, 10=3G



(* equivalent_register_removal = "no" *)
(* KEEP = "TRUE" *)
reg  [2:0]  tx2_ce = 2'b00;                         // TX clock enable for SD mode
reg  [10:0] tx2_ce_gen = 11'b00000100001;           // Generates 5/6/5/6 cadence for TX SD clock enable
wire        tx2_ce_mux;                             // Used to generate tx2_ce
reg  [1:0]  tx2_mode_int = 2'b00;                   // tx2_mode generated in rx2_usrclk domain
reg  [1:0]  tx2_mode_sync1 = 2'b00;                 // Used to sync tx2_mode_int to FMC clock
reg  [1:0]  tx2_mode_sync2 = 2'b00;                 // This version of tx2_mode is synchronous with clk_fmc_27M// 

//////// 		MGT		////////

wire        rx2_resetdone;                          // MGT RXRESETDONE
wire [2:0]  rx2_bufstatus;                          // MGT RXBUFSTATUS
wire        rx2_ratedone;                           // MGT RXRATEDONE
wire        rx2_cdrreset;                           // MGT RXCDRRESET
wire        rx2_bufreset;                           // MGT RXBUFRESET
wire [1:0]  rx2_rate;                               // Control MGT RXRATE port
wire [19:0] rx2_gtx_data;                           // Raw received data from MGT RX
wire        tx2_resetdone;                          // 1 = MGT TX reset done
wire [1:0]  tx2_bufstatus;                          // MGT TXBUFSTATUS output
wire        tx2_pll_locked;                         // 1 = TX PMA PLL locked to reference clock
wire        tx2_reset;                              // Connects to MGT TXRESET input 
wire [19:0] tx2_gtx_data;                           // Encoded data stream to MGT TX
wire [12:0] tx2_gtxtest;                            // GTXTEST
wire [15:0] dp1_do;
wire        dp1_drdy;
wire [7:0]  dp1_daddr;
wire [15:0] dp1_di;
wire        dp1_den;
wire        dp1_dwe;
wire        rx2_pll_locked;                         // MGT RX PMA PLL locked to reference clock
wire        tx2_slew;                               // Cable driver slew rate control
wire        gtxtxreset;                             // GTXTXRESET
wire [4:0]  tx2_postemphasis;                       // GTX TX postemphasis

wire        drpclk;  
wire        mgtclk_rx;                              // RX MGT reference clock
wire        mgtclk_tx;                              // TX MGT reference clock

/////////////////////	Video Generator Wires	////////////////////////////

wire  [9:0]		y_tx;          // luma output channel
wire	[9:0]  	c_tx;          // chroma output channel
//wire        	h_blank;    // asserted during horizontal blanking period
//wire        	v_blank;    // asserted 
//wire        	field;      // indicates current field
//wire        	trs;        // asserted during 4 words of TRS symbol,
//wire        	xyz;        // asserted during TRS XYZ word
wire  [10:0] 	tx_line_num;    // current vertical line number

///////////////////////////////////////////////////////////////////////////
 
wire  [9:0]		y_rx;          // luma output channel
wire	[9:0]  	c_rx;          // chroma output channel

wire				CLK_27_MHZ_P; 		// 27 MHz signal for pass demo clk input
wire				CLK_27_MHZ_N; 		// 27 MHz signal for pass demo clk input

wire				internal_connection_P; 		// mgt internal serial connection P
wire				internal_connection_N; 		// mgt internal serial connection N
                               // 32 MHz DRP clock
// In SD mode, output the 27 MHz clock enable to Si5324 B, otherwise output rx2_usrclk.
// Si5324 B will synthesize a 148.5 MHz or 148.35 MHz clock from this input reference
// clock to provide a reference clock to the GTX TX.
//
//OBUFDS # (
//    .IOSTANDARD ("LVDS_25"))
//SI5324B_CLKIN2 (
//    .I          (rx2_mode_SD ? rx2_ce : rx2_usrclk),
//    .O          (FMC_HPC_LA16_P),
//    .OB         (FMC_HPC_LA16_N));

OBUFDS # (
    .IOSTANDARD ("LVDS_25"))
CLK_27_MHZ (
    .I          (DRP_CLK_IN),
    .O          (CLK_27_MHZ_P),
    .OB         (CLK_27_MHZ_N));

//
// Global clock buffer for recovered clock
//
BUFG RXBUFG (
    .I              (rx2_recclk),
    .O              (rx2_usrclk));
//

//
// Reclocking buffer for SD
//
BUFGMUX TXUSRCLK_BUFG (
    .I0     (rx2_recclk),
    .I1     (tx2_outclk),
    .S      (rx2_mode_SD),
    .O      (tx2_usrclk));


// 148.5 MHz MGT RX reference clock input
IBUFDS_GTXE1 MGTCLKIN0 (
    .I          (Q0_CLK1_MGTREFCLK_PAD_P_IN),
    .IB         (Q0_CLK1_MGTREFCLK_PAD_N_IN),
    .CEB        (1'b0),
    .O          (mgtclk_rx),
    .ODIV2      ());

// 148.X MHz MGT TX reference clock input
IBUFDS_GTXE1 MGTCLKIN1 (
    .I          (Q1_CLK0_MGTREFCLK_PAD_P_IN),
    .IB         (Q1_CLK0_MGTREFCLK_PAD_N_IN),
    .CEB        (1'b0),
    .O          (mgtclk_tx),
    .ODIV2      ());


///////////////////////////// Component to test/////////////////


encoder dut (
		
		.FMC_HPC_DP1_C2M_N			(RXN_IN), 							//  output SDI TX2
		.FMC_HPC_DP1_C2M_P			(RXP_IN), 							//  output SDI TX2
		.FMC_HPC_DP1_M2C_N			(TXN_OUT), 							//  input  SDI RX2
		.FMC_HPC_DP1_M2C_P			(TXP_OUT), 							//  input  SDI RX2

		.FMC_HPC_GBTCLK0_M2C_N		(Q1_CLK0_MGTREFCLK_PAD_N_IN),		//  input Q113 MGTREFCLK0 (148.5 MHz RX refclk) 
		.FMC_HPC_GBTCLK0_M2C_P		(Q1_CLK0_MGTREFCLK_PAD_P_IN), 	//  input Q113 MGTREFCLK0 (148.5 MHz RX refclk)
		.FMC_HPC_CLK2_M2C_MGT_C_N	(Q0_CLK1_MGTREFCLK_PAD_N_IN), 	//  input Q113 MGTREFCLK1 (148.X MHz TX refclk)
		.FMC_HPC_CLK2_M2C_MGT_C_P	(Q0_CLK1_MGTREFCLK_PAD_P_IN), 	//  input Q113 MGTREFCLK1 (148.X MHz TX refclk)
	
		.FMC_HPC_HB06_CC_N			(CLK_27_MHZ_N),  						//  input 27 MHz Clock from FMC
		.FMC_HPC_HB06_CC_P			(CLK_27_MHZ_P),  						//  input 27 MHz Clock from FMC

		.FMC_HPC_LA00_CC_P			(), 										//  output main SPI interface SCK
		.FMC_HPC_LA00_CC_N			(), 										//  output main SPI interface SCK
		
		.FMC_HPC_LA27_P				(), 										//  input  main SPI interface MISO
		.FMC_HPC_LA14_N				(), 	 									//  output main SPI interface SS
		
		.FMC_HPC_LA29_P				(),	 									//  output CML SPI interface SCK
		.FMC_HPC_LA29_N				(),	 									//  input  CML SPI interface MISO
		.FMC_HPC_LA07_P				(),										//  output CML SPI interface MOSI
		.FMC_HPC_LA07_N				(),	 									//  output CML SPI interface SS

		.FMC_HPC_LA13_P				(), 										//  output Si5324 A reset asserted low
		.FMC_HPC_LA33_P				(), 										//  output Si5324 B reset asserted low
		.FMC_HPC_LA26_P				(), 										//  output Si5324 C reset asserted low
		
		.FMC_HPC_LA16_P				(),	 									//  output RXRECCLK (HD/3G modes) or
		.FMC_HPC_LA16_N				(),	 									//  output SD-SDI DRU ready to CML Si5324 B CKIN2
				
		// Misc ML605 IO
		
		.GPIO_LED_0						(), 										//  output Display RX locked status on this LED
		.USER_SMA_GPIO_P				(), 										//  output For testing purposes
		.USER_SMA_GPIO_N				(), 										//  output TXUSRCLK on the ML605 GPIO SMA connectors
		.LCD_DB4							(),										//  output
		.LCD_DB5							(),										//  output
		.LCD_DB6							(),										//  output
		.LCD_DB7							(),										//  output
		.LCD_E							(), 										//  output
		.LCD_RW							(),										//  output
		.LCD_RS							(),										//  output
		.GPIO_SW_C						(), 										//  input
		.GPIO_SW_W						(), 										//  input
		.GPIO_SW_E						(), 										//  input
		.GPIO_SW_N						(),										//  input
		.GPIO_SW_S						(),										//  input
		
	///////////////////  Debug Input please comment. ////////////////////////	
		.SIM_RESET						(GTXTXRESET_IN),						// input added to reset from tesbench
		.rx_trs_sim						(rx_trs), 
		.rx_xyz_sim						(rx_xyz), 
		.rx_line_num_check			(rx_line_num_check),
////////////////////////////////////////////////////////////////////

// Mig Memory IP external connection to ML605 Board    

		.clk_ref_p     				(MIG_REF_CLK_P),		// input differential iodelayctrl clk 
		.clk_ref_n						(MIG_REF_CLK_N),		// input
		.ddr3_dq							(), 		// inout  [DQ_WIDTH-1:0]   
		.ddr3_addr						(),   	// output [ROW_WIDTH-1:0] 
		.ddr3_ba							(),		// output [BANK_WIDTH-1:0]
		.ddr3_ras_n						(),		// output  
		.ddr3_cas_n						(),   	// output  
		.ddr3_we_n						(),   	// output  
		.ddr3_reset_n					(),   	// output  
		.ddr3_cs_n						(),   	// output [(CS_WIDTH*nCS_PER_RANK)-1:0]
		.ddr3_odt						(),		// output [(CS_WIDTH*nCS_PER_RANK)-1:0]
		.ddr3_cke						(),   	// output [CKE_WIDTH-1:0]
		.ddr3_dm							(),		// output [DM_WIDTH-1:0] 
		.ddr3_dqs_p						(),		// inout  [DQS_WIDTH-1:0]
		.ddr3_dqs_n						(),		// inout  [DQS_WIDTH-1:0]
		.ddr3_ck_p						(rx_clk), 		// output [CK_WIDTH-1:0]
		.ddr3_ck_n						(tx_clk),		// output [CK_WIDTH-1:0]
		.error							(),		// output
		.phy_init_done					(),		// output
		.pll_lock						(),   	// output ML605 GPIO LED
		.heartbeat						(),  		// output ML605 GPIO LED
		.sys_rst							(GTXTXRESET_IN)  	// input System reset	 
//	output                           	   test     // ML605 GPIO LED 	 



	
	);


// Verilog 2001 initialization code VROM
// Created by multigenHD_romgen.v
// Video format mapping:
//   0 =  SMPTE 296M - 720p   50Hz                   
//   1 =  SMPTE 274M - 1080sF 24Hz & 23.98Hz         
//   2 =  SMPTE 274M - 1080i  30Hz & 29.97 Hz        
//   3 =  SMPTE 274M - 1080i  25Hz                   
//   4 =  SMPTE 274M - 1080p  30Hz & 29.97Hz         
//   5 =  SMPTE 274M - 1080p  25Hz                   
//   6 =  SMPTE 274M - 1080p  24Hz & 23.98Hz         
//   7 =  SMPTE 296M - 720p   60Hz & 59.94Hz   

//    user_opt Parameters
            
//			2'b01: h_region = HRGN_USROPT1;
//			2'b10: h_region = HRGN_USROPT2;
//       2'b11: h_region = HRGN_USROPT3;


multigenHD video_gen (
    
	 .clk 					(tx2_usrclk), //(mgtclk_tx),		// word-rate clock -- input wire 
    .rst						(GTXTXRESET_IN),//(1'b0),		// async reset -- input wire 
    .ce						(1'b1),		// clock enable -- input wire 
    .std						(3'b000),		// selects video format  -- input wire [2:0]
    .pattern				(2'b00),		// 00 = RP 219 colorbars, X1 = RP 198 checkfield, 10 = 75% colorbars  -- input wire [1:0]  
    .user_opt				(2'b01),		// selects option for the *2 & *3 blocks of RP 219 -- input wire [1:0]  
    .y						(y_tx),		// luma output channel -- output  reg  [9:0]
    .c						(c_tx),		// chroma output channel -- output  reg  [9:0]  
    .h_blank				(h_blank),		// asserted during horizontal blanking period -- output  wire
    .v_blank				(v_blank),  		// asserted -- output  wire        
    .field					(field),	   // indicates current field -- output  wire        
    .trs						(trs),		// asserted during 4 words of TRS symbol -- output  wire 
    .xyz						(xyz),		// asserted during TRS XYZ word -- output  wire        
    .line_num 				(tx_line_num)			// current vertical line number -- output  reg  [10:0] 
);


triple_sdi_rxtx_top gtx_sdi (

	.rx_rst										(1'b0),								// input rx_rst
	.rx_usrclk									(rx2_usrclk), 						// input rx_usrclk
	.rx_frame_en								(1'b1), 								// input rx_frame_en
	.rx_mode_en									(3'b111), 							// input [2 : 0] rx_mode_en
	.rx_mode										(rx2_mode), 						// output [1 : 0] rx_mode used in always
	.rx_mode_HD									(rx2_mode_HD), 					// output rx_mode_HD
	.rx_mode_SD									(rx2_mode_SD), 					// output rx_mode_SD
	.rx_mode_3G									(rx2_mode_3G),						// output rx_mode_3G
	.rx_mode_locked							(rx2_mode_locked), 				// output rx_mode_locked
	.rx_bit_rate								(rx2_m), 							// output rx_bit_rate
	.rx_t_locked								(rx2_locked), 						// output rx_t_locked -- out to LED
	.rx_t_family								(rx2_t_family), 					// output [3 : 0] rx_t_family -- recived signal info
	.rx_t_rate									(rx2_t_rate), 						// output [3 : 0] rx_t_rate -- recived signal info
	.rx_t_scan									(rx2_t_scan), 						// output rx_t_scan -- recived signal info 
	.rx_level_b_3G								(rx2_level_b), 					// output rx_level_b_3G
	.rx_ce_sd									(rx2_ce), 							// output rx_ce_sd -- enable SD Clock signal
	.rx_nsp										(), 									// output rx_nsp
	.rx_line_a									(rx2_ln_a), 						// output [10 : 0] -- received line number chA
	.rx_a_vpid									(rx2_a_vpid), 						// output [31 : 0] -- NO Connected in demo just wire
	.rx_a_vpid_valid							(rx2_a_vpid_valid), 				// output 			 -- NO Connected in demo just wire
	.rx_b_vpid									(rx2_b_vpid), 						// output [31 : 0] rx_b_vpid
	.rx_b_vpid_valid							(rx2_b_vpid_valid), 				// output rx_b_vpid_valid
	.rx_crc_err_a								(rx2_crc_err_a), 					// output rx_crc_err_a  -- wire flag for crc chA error
	.rx_ds1a										(y_rx), 									// output [9 : 0] Received LUMA chA
	.rx_ds2a										(c_rx), 									// output [9 : 0] Received Chromas chA
	.rx_eav										(rx2_eav), 							// output rx_eav
	.rx_sav										(rx2_sav), 							// output rx_sav
	.rx_trs										(rx2_trs), 							// output rx_trs
	.rx_line_b									(rx2_line_b), 						// output [10 : 0] -- received line number chB
	.rx_dout_rdy_3G							(rx2_dout_rdy_3G), 				// output rx_dout_rdy_3G -- 3G Clock Enable
	.rx_crc_err_b								(rx2_crc_err_b), 					// output rx_crc_err_a  -- wire flag for crc chB error
	.rx_ds1b										(rx2_ds1b),							// output [9 : 0] Received LUMA chB (3G-B only)
	.rx_ds2b										(rx2_ds2b), 						// output [9 : 0] Received Chromas chA (3G-B only)
	.rx_edh_errcnt_en							(16'b0_00001_00001_00000),		// input [15 : 0] rx_edh_errcnt_en                     
	.rx_edh_clr_errcnt						(rx2_clr_errs), 					// input User control to clear RX CRC & EDH error counts
	.rx_edh_ap									(), 									// output rx_edh_ap
	.rx_edh_ff									(), 									// output rx_edh_ff
	.rx_edh_anc									(), 									// output rx_edh_anc
	.rx_edh_ap_flags							(), 									// output [4 : 0] rx_edh_ap_flags
	.rx_edh_ff_flags							(), 									// output [4 : 0] rx_edh_ff_flags
	.rx_edh_anc_flags							(), 									// output [4 : 0] rx_edh_anc_flags
	.rx_edh_packet_flags						(), 									// output [3 : 0] rx_edh_packet_flags
	.rx_edh_errcnt								(rx2_edh_errcnt), 				// output [15 : 0]  SD-SDI EDH error count
	.rx_recclk_txdata							(), 									// output [19 : 0] rx_recclk_txdata
  
	.tx_rst										(1'b0),								// input tx_rst
	.tx_usrclk									(tx2_usrclk), 						// input tx_usrclk
	.tx_ce										(tx2_ce), 							// input [2 : 0] tx_ce
	.tx_din_rdy									(tx2_din_rdy), 					// input tx_din_rdy 3g Clock enable
	.tx_mode										(tx2_mode), 						// input [1 : 0] tx_mode
	.tx_rate_change_done						(), 									// output tx_rate_change_done unconnected
	.tx_level_b_3G								(rx2_level_b_sync), 				// input tx_level_b_3G
	.tx_insert_crc								(1'b0), 								// input tx_insert_crc
	.tx_insert_ln								(1'b1), 								// input tx_insert_ln
	.tx_insert_edh								(1'b0), 								// input tx_insert_edh
	.tx_insert_vpid							(1'b0), 								// input tx_insert_vpid
	.tx_overwrite_vpid						(1'b0),								// input tx_overwrite_vpid
	.tx_video_a_y_in							(y_tx), 								// input [9 : 0] Y Video components ch1
	.tx_video_a_c_in							(c_tx), 								// input [9 : 0] C Video components ch1
	.tx_video_b_y_in							(), 									// input [9 : 0] Y Video components ch2
	.tx_video_b_c_in							(), 									// input [9 : 0] C Video components ch2
	.tx_line_a									(tx_line_num), //(11'b0),		// input [10 : 0] tx_line_a
	.tx_line_b									(11'b0), 							// input [10 : 0] tx_line_b
	.tx_vpid_byte1								(8'h00), 							// input [7 : 0] tx_vpid_byte1
	.tx_vpid_byte2								(8'h00), 							// input [7 : 0] tx_vpid_byte2
	.tx_vpid_byte3								(8'h00), 							// input [7 : 0] tx_vpid_byte3
	.tx_vpid_byte4a							(8'h00), 							// input [7 : 0] tx_vpid_byte4a
	.tx_vpid_byte4b							(8'h00), 							// input [7 : 0] tx_vpid_byte4b
	.tx_vpid_line_f1							(11'd0), 							// input [10 : 0] tx_vpid_line_f1
	.tx_vpid_line_f2							(11'd0),						 		// input [10 : 0] tx_vpid_line_f2
	.tx_vpid_line_f2_en						(1'b0), 								// input tx_vpid_line_f2_en
	.tx_ds1a_out								(), 									// output [9 : 0] tx_ds1a_out
	.tx_ds2a_out								(), 									// output [9 : 0] tx_ds2a_out
	.tx_ds1b_out								(), 									// output [9 : 0] tx_ds1b_out
	.tx_ds2b_out								(), 									// output [9 : 0] tx_ds2b_out
	.tx_use_dsin								(1'b0), 								// input tx_use_dsin
	.tx_ds1a_in									(10'b0), 							// input [9 : 0] tx_ds1a_in
	.tx_ds2a_in									(10'b0), 							// input [9 : 0] tx_ds2a_in
	.tx_ds1b_in									(10'b0),								// input [9 : 0] tx_ds1b_in
	.tx_ds2b_in									(10'b0), 							// input [9 : 0] tx_ds2b_in
  
	.gtx_rxresetdone							(rx2_resetdone), 					// input from GTX
	.gtx_rxbufstatus2							(rx2_bufstatus[2]), 				// input from GTX
	.gtx_rxratedone							(rx2_ratedone), 					// input from GTX
	.gtx_rxcdrreset							(rx2_cdrreset), 					// output to GTX
	.gtx_rxbufreset							(rx2_bufreset), 					// output to GTX
	.gtx_rxrate									(rx2_rate), 						// output [1 : 0] to gtx
	.gtx_rxdata									(rx2_gtx_data), 					// input [19 : 0] from gtx
	.gtx_txreset_in							(1'b0), 								// input gtx_txreset_in
	.gtx_txresetdone							(tx2_resetdone), 					// input from gtx
	.gtx_txbufstatus1							(tx2_bufstatus[1]), 				// input from gtx
	.gtx_txplllkdet							(gtx_txplllkdet), 				// input from gtx
	.gtx_txreset								(),//(tx2_reset), 						// output to gtx
	.gtx_txdata									(tx2_gtx_data), 					// output [19 : 0] to gtx
	.gtx_gtxtest								(tx2_gtxtest), 					// output [12 : 0] to gtx
	.gtx_drpclk									(drpclk), 							// input gtx_drpclk
	.gtx_drpdo									(dp1_do), 							// input [15 : 0] from gtx
	.gtx_drdy									(dp1_drdy), 						// input gtx_drdy
	.gtx_daddr									(dp1_daddr), 						// output [7 : 0] to gtx
	.gtx_di										(dp1_di), 							// output [15 : 0] to gtx
	.gtx_den										(dp1_den), 							// output to gtx
	.gtx_dwe										(dp1_dwe), 							// output to gtx
  
	.tx_ce_align_err							(), 									// output tx_ce_align_err
	.tx_slew										(tx2_slew), 						// output tx_slew
	.rx_mode_test								(2'b00), 							// input [1 : 0] rx_mode_test
	.test											(3'b000) 							// input [2 : 0] test
);



//////////////////////////////////////////////////
//		Reclock Management for SD and 3G modes		//
//////////////////////////////////////////////////

/*
assign rx2_crc_err = ~rx2_mode_SD & (rx2_crc_err_a | (rx2_mode_3G & rx2_level_b & rx2_crc_err_b));

assign GPIO_LED_0 = rx2_locked;

//
// Capture CRC errors for the ChipScope VIO. This FF is manually cleared by
// a VIO module output.
//
always @ (posedge rx2_usrclk)
    if (rx2_clr_errs)
        rx2_crc_err_capture <= 1'b0;
    else if (rx2_crc_err)
        rx2_crc_err_capture <= 1'b1;
    
//------------------------------------------------------------------------------
// RX -> TX Interface Logic
//
// In HD and 3G modes, the data from the RX is passed to the TX data path with
// just a set of registers for timing purposes. In SD mode, the data passes
// through a FIFO to synchronize it to the TX clock. This is necessary because,
// in SD mode, the RX data is not timed to a recovered clock. Instead, the RX
// clock is really derived directly from the GTX reference clock and the cadence
// of the RX clock enable varies slightly when the DRU needs to catch up. The
// TX side, however, is driven by a recovered clock synthesized by the Si5324
// and the cadence of TX clock enable never varies from 5/6/5/6 in SD mode.
// 

//
// Only change the tx2_mode signal when rx2_mode_locked is asserted. This 
// insures that the tx2_mode does change as the RX is unlocked and searching
// for the correct SDI mode. 
//

always @ (posedge rx2_usrclk)
    if (rx2_mode_locked)
        tx2_mode_int <= rx2_mode;

always @ (posedge tx2_usrclk)
begin
    tx2_mode_x <= tx2_mode_int;
    tx2_mode <= tx2_mode_x;
end

always @ (posedge tx2_usrclk)
begin
    rx2_mode_SD_syncer <= {rx2_mode_SD_syncer[0], rx2_mode_SD};
    rx2_mode_3G_syncer <= {rx2_mode_3G_syncer[0], rx2_mode_3G};
    rx2_level_b_syncer <= {rx2_level_b_syncer[0], rx2_level_b};
end

assign rx2_mode_SD_sync = rx2_mode_SD_syncer[1];
assign rx2_mode_3G_sync = rx2_mode_3G_syncer[1];
assign rx2_level_b_sync = rx2_level_b_syncer[1];

//
// Link A data stream 2 and both data streams for link B are always directly
// driven by the data streams from the receiver.
//
always @ (posedge tx2_usrclk)
    if (rx2_dout_rdy_3G) 
    begin
        tx2_ds2a <= rx2_ds2a;
        tx2_ds1b <= rx2_ds1b;
        tx2_ds2b <= rx2_ds2b;
    end

//
// Link A data stream 1 is driven by the data stream from receiver in HD and 3G
// modes, but in SD mode, the source is the output of the video_fifo.
// 
always @ (posedge tx2_usrclk)
    if (rx2_mode_SD_sync)
    begin
        tx2_ds1a <= fifo2_dout;
        tx2_eav <= 1'b0;
        tx2_sav <= 1'b0;
    end
    else if (rx2_dout_rdy_3G)
    begin
        tx2_ds1a <= rx2_ds1a;
        tx2_eav <= rx2_eav;
        tx2_sav <= rx2_sav;
    end

// 
// FIFO for SD-SDI data path -- 16 locations deep -- asynchronous RD/WR clocks
//
// This FIFO is used only in SD mode to allow for the phase differences between
// the receive clock and the transmit clock. Data is written to the FIFO using
// the rx2_usrclk, which in SD mode is locked to the RX reference clock and
// enabled using the rx2_ce clock enable. Data is read from the FIFO using
// tx2_usrclk, driven by TXOUTCLK in SD mode, and enabled using the tx2_ce
// clock enabled which has a constant 5/6/5/6 cadence in SD mode.
//
// This FIFO was generated using the Corgen FIFO generator. It uses distributed
// RAM. The programmable full signal is set to 8.
// */


 //v6sdi_wrapper #
	v6sdi_wrapper_sim #	
	(
        .WRAPPER_SIM_GTXRESET_SPEEDUP   (1)//(0)      // Set this to 1 for simulation
    )
    v6sdi_wrapper_i
    (
        //_____________________________________________________________________
        //_____________________________________________________________________
        //GTX0  (X0Y0)
		  .GTX0_DOUBLE_RESET_CLK_IN 			(drpclk),
        //---------------------- Loopback and Powerdown Ports ----------------------
        .GTX0_LOOPBACK_IN               	(3'b000),
        //----------------- Receive Ports - RX Data Path interface -----------------
        .GTX0_RXDATA_OUT                	(rx2_gtx_data),
        .GTX0_RXRECCLK_OUT              	(rx2_recclk),  // to buffer and to triple rate sdi
        .GTX0_RXRESET_IN                	(1'b0),
        .GTX0_RXUSRCLK2_IN              	(rx2_usrclk),
        //----- Receive Ports - RX Driver,OOB signalling,Coupling and Eq.,CDR ------
        .GTX0_RXN_IN                    	(RXN_IN),	// General N input from Mgt
        .GTX0_RXP_IN                    	(RXP_IN),	// General P input from Mgt
        //------ Receive Ports - RX Elastic Buffer and Phase Alignment Ports -------
        .GTX0_RXBUFRESET_IN             	(rx2_bufreset),
        .GTX0_RXBUFSTATUS_OUT           	(rx2_bufstatus),
        .GTX0_RXCDRRESET_IN             	(rx2_cdrreset),
        //---------------------- Receive Ports - RX PLL Ports ----------------------
        .GTX0_MGTREFCLKRX_IN				 	({1'b0, mgtclk_rx}),
        .GTX0_PERFCLKRX_IN              	(1'b0),
		  .GTX0_GREFCLKRX_IN              	(1'b0),
        .GTX0_GTXRXRESET_IN             	(GTXRXRESET_IN),   //   Managed from Test bench
		  .GTX0_NORTHREFCLKRX_IN				(2'b00),
		  .GTX0_SOUTHREFCLKRX_IN  				(2'b00),
		  .GTX0_RXPLLREFSELDY_IN				(3'b000),
        .GTX0_PLLRXRESET_IN             	(1'b0),
        .GTX0_RXPLLLKDET_OUT            	(rx2_pll_locked),    // out to async in
        .GTX0_RXRATE_IN                 	(rx2_rate),
        .GTX0_RXRATEDONE_OUT            	(rx2_ratedone),
        .GTX0_RXRESETDONE_OUT           	(rx2_resetdone),

        //----------- Shared Ports - Dynamic Reconfiguration Port (DRP) ------------
        .GTX0_DADDR_IN                  (dp1_daddr),
        .GTX0_DCLK_IN                   (drpclk),
        .GTX0_DEN_IN                    (dp1_den),
        .GTX0_DI_IN                     (dp1_di),
        .GTX0_DRDY_OUT                  (dp1_drdy),
        .GTX0_DRPDO_OUT                 (dp1_do),
        .GTX0_DWE_IN                    (dp1_dwe),
        //----------------------- Transmit Ports - GTX Ports -----------------------
        .GTX0_GTXTEST_IN                (tx2_gtxtest),
        //---------------- Transmit Ports - TX Data Path interface -----------------
        .GTX0_TXDATA_IN                 (tx2_gtx_data),
        .GTX0_TXOUTCLK_OUT              (tx2_outclk),
        .GTX0_TXRESET_IN                (tx2_reset),
        .GTX0_TXUSRCLK2_IN              (tx2_usrclk),
        //-------------- Transmit Ports - TX Driver and OOB signaling --------------
        .GTX0_TXN_OUT                   (TXN_OUT),//(internal_connection_N),
        .GTX0_TXP_OUT                   (TXP_OUT),//(internal_connection_P),
        .GTX0_TXPOSTEMPHASIS_IN         (tx2_postemphasis),
        //--------- Transmit Ports - TX Elastic Buffer and Phase Alignment ---------
        .GTX0_TXBUFSTATUS_OUT           (tx2_bufstatus),
        //--------------------- Transmit Ports - TX PLL Ports ----------------------
        .GTX0_MGTREFCLKTX_IN            ({mgtclk_tx, 1'b0}),
		  .GTX0_PERFCLKTX_IN              (1'b0),
		  .GTX0_GREFCLKTX_IN              (1'b0),
        .GTX0_NORTHREFCLKTX_IN          (2'b00),
        .GTX0_SOUTHREFCLKTX_IN          (2'b00), 
		  .GTX0_GTXTXRESET_IN             (GTXTXRESET_IN),//(gtxtxreset),     // Keep TX reset when reference clock is not stable,      
		  .GTX0_TXPLLREFSELDY_IN  			 (3'b001),			// change gtxtxreset to GTXTXRESET_IN, to manage from testbench
        .GTX0_PLLTXRESET_IN             (1'b0),
        .GTX0_TXPLLLKDET_OUT            (tx2_pll_locked),
        .GTX0_TXRESETDONE_OUT           (tx2_resetdone)


    );


//
// Generate a TX clock enable that has a 5/6/5/6 cadence in SD mode and is 
// always High in HD and 3G modes.
//

assign rx2_mode_SD_sync = 1'b0;  // Important!!! The following lines generate the transmission clock enable!!!

always @ (posedge tx2_usrclk)
    tx2_ce_gen <= {tx2_ce_gen[9:0], tx2_ce_gen[10]};

assign tx2_ce_mux = rx2_mode_SD_sync ? tx2_ce_gen[10] : 1'b1;

always @ (posedge tx2_usrclk)
    tx2_ce <= {3 {tx2_ce_mux}};  // Important!!! The lines above generate the transmission clock enable!!!


// Hold the TX in reset till the TX user clocks are stable  --- From Xilinx gtx top Testbench 
    assign tx2_reset = !tx2_pll_locked;

assign drpclk = DRP_CLK_IN;   // Internal clock, in the proyect it works @ 27 MHz In Simulation is 50MHz and is working

//
// The tx2_din_rdy signal is controlled by the rx2_dout_rdy_3G signal. In all
// modes except 3G-B, it will always be High. In 3G-B mode, it is asserted every
// other clock cycle.
//

assign tx2_postemphasis = (tx2_mode == 2'b10) ? 5'd20 : 5'd10;

assign tx2_din_rdy = rx2_dout_rdy_3G;

assign TRACK_DATA_OUT = tx2_gtx_data;//{y_tx, c_tx};

assign y_tx_check = y_tx; //y_rx;      
assign c_tx_check = c_tx;// c_rx;
assign y_rx_check = y_rx;      
assign c_rx_check = c_rx;

assign tx_line_num_check = tx_line_num;
//assign rx_clk = rx2_usrclk;// tx2_reset;//mgtclk_tx;//drpclk;//rx2_recclk;//mgtclk_rx;
//assign tx_clk = tx2_usrclk;//mgtclk_tx;



endmodule
