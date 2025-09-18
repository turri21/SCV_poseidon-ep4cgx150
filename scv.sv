//============================================================================
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//============================================================================

module guest_top
(
	input         CLOCK_27,
`ifdef USE_CLOCK_50
	input         CLOCK_50,
`endif

	output        LED,
	output [VGA_BITS-1:0] VGA_R,
	output [VGA_BITS-1:0] VGA_G,
	output [VGA_BITS-1:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,

`ifdef USE_HDMI
	output        HDMI_RST,
	output  [7:0] HDMI_R,
	output  [7:0] HDMI_G,
	output  [7:0] HDMI_B,
	output        HDMI_HS,
	output        HDMI_VS,
	output        HDMI_PCLK,
	output        HDMI_DE,
	inout         HDMI_SDA,
	inout         HDMI_SCL,
	input         HDMI_INT,
`endif

	input         SPI_SCK,
	inout         SPI_DO,
	input         SPI_DI,
	input         SPI_SS2,    // data_io
	input         SPI_SS3,    // OSD
	input         CONF_DATA0, // SPI_SS for user_io

`ifdef USE_QSPI
	input         QSCK,
	input         QCSn,
	inout   [3:0] QDAT,
`endif
`ifndef NO_DIRECT_UPLOAD
	input         SPI_SS4,
`endif

	output [12:0] SDRAM_A,
	inout  [15:0] SDRAM_DQ,
	output        SDRAM_DQML,
	output        SDRAM_DQMH,
	output        SDRAM_nWE,
	output        SDRAM_nCAS,
	output        SDRAM_nRAS,
	output        SDRAM_nCS,
	output  [1:0] SDRAM_BA,
	output        SDRAM_CLK,
	output        SDRAM_CKE,

`ifdef DUAL_SDRAM
	output [12:0] SDRAM2_A,
	inout  [15:0] SDRAM2_DQ,
	output        SDRAM2_DQML,
	output        SDRAM2_DQMH,
	output        SDRAM2_nWE,
	output        SDRAM2_nCAS,
	output        SDRAM2_nRAS,
	output        SDRAM2_nCS,
	output  [1:0] SDRAM2_BA,
	output        SDRAM2_CLK,
	output        SDRAM2_CKE,
`endif

	output        AUDIO_L,
	output        AUDIO_R,
`ifdef I2S_AUDIO
	output        I2S_BCK,
	output        I2S_LRCK,
	output        I2S_DATA,
`endif
`ifdef I2S_AUDIO_HDMI
	output        HDMI_MCLK,
	output        HDMI_BCK,
	output        HDMI_LRCK,
	output        HDMI_SDATA,
`endif
`ifdef SPDIF_AUDIO
	output        SPDIF,
`endif
`ifdef USE_AUDIO_IN
	input         AUDIO_IN,
`endif
`ifdef USE_MIDI_PINS
	output        MIDI_OUT,
	input         MIDI_IN,
`endif
`ifdef SIDI128_EXPANSION
	input         UART_CTS,
	output        UART_RTS,
	inout         EXP7,
	inout         MOTOR_CTRL,
`endif
	input         UART_RX,
	output        UART_TX
);

`ifdef NO_DIRECT_UPLOAD
localparam bit DIRECT_UPLOAD = 0;
wire SPI_SS4 = 1;
`else
localparam bit DIRECT_UPLOAD = 1;
`endif

`ifdef USE_QSPI
localparam bit QSPI = 1;
assign QDAT = 4'hZ;
`else
localparam bit QSPI = 0;
`endif

`ifdef VGA_8BIT
localparam VGA_BITS = 8;
`else
localparam VGA_BITS = 6;
`endif

`ifdef USE_HDMI
localparam bit HDMI = 1;
assign HDMI_RST = 1'b1;
`else
localparam bit HDMI = 0;
`endif

`ifdef BIG_OSD
localparam bit BIG_OSD = 1;
`define SEP "-;",
`else
localparam bit BIG_OSD = 0;
`define SEP
`endif

// remove this if the 2nd chip is actually used
`ifdef DUAL_SDRAM
assign SDRAM2_A = 13'hZZZZ;
assign SDRAM2_BA = 0;
assign SDRAM2_DQML = 0;
assign SDRAM2_DQMH = 0;
assign SDRAM2_CKE = 0;
assign SDRAM2_CLK = 0;
assign SDRAM2_nCS = 1;
assign SDRAM2_DQ = 16'hZZZZ;
assign SDRAM2_nCAS = 1;
assign SDRAM2_nRAS = 1;
assign SDRAM2_nWE = 1;
`endif

`ifdef USE_HDMI
wire        i2c_start;
wire        i2c_read;
wire  [6:0] i2c_addr;
wire  [7:0] i2c_subaddr;
wire  [7:0] i2c_dout;
wire  [7:0] i2c_din;
wire        i2c_ack;
wire        i2c_end;
`endif

import scv_pkg::hmi_t;
import scv_pkg::mapper_t;

`include "build_id.v" 
localparam CONF_STR = {
	"SCV;;",
	"-;",
    "F1,ROMBIN;",
	"-;",

	//"O[2],TV Mode,NTSC,PAL;",
	"O8,Palette,RGB,RF;",
   "O9A,Overscan mask,Large,Small,None;",
	"-;",
//    "O[19:16],Cartridge mapper,automatic,rom8k,rom16k,rom32k,rom32k_ram,rom64k,rom128k,rom128_ram;",
	"-;",
	"T0,Reset;",
//	"J,Trig 1,Trig 2,SELECT 1,SELECT 2,SELECT 3,SELECT 4,SELECT EN;",
//	"jn,A,B,X,Y,L,R,Start;",
	"V,Poseidon-",`BUILD_DATE
};

// Status and control signals
wire [31:0] status;
wire  [1:0] buttons;
wire  [1:0] scandoubler_disable;
wire        ypbpr;
wire        no_csync;



//////////////////////////////////////////////////////////////////////
// Download manager

wire         rominit_active;
wire         rominit_sel_boot, rominit_sel_chr, rominit_sel_apu,
             rominit_sel_cart;
wire [16:0]  rominit_addr;
wire [7:0]   rominit_data;
wire         rominit_valid;

rominit rominit
  (
   .CLK_SYS(clk_sys),

   .IOCTL_DOWNLOAD(ioctl_download),
   .IOCTL_INDEX(ioctl_index),
   .IOCTL_WR(ioctl_wr),
   .IOCTL_ADDR(ioctl_addr),
   .IOCTL_DOUT(ioctl_data),
   .IOCTL_WAIT(ioctl_wait),

   .ROMINIT_ACTIVE(rominit_active),
   .ROMINIT_SEL_BOOT(rominit_sel_boot),
   .ROMINIT_SEL_CHR(rominit_sel_chr),
   .ROMINIT_SEL_APU(rominit_sel_apu),
   .ROMINIT_SEL_CART(rominit_sel_cart),
   .ROMINIT_ADDR(rominit_addr),
   .ROMINIT_DATA(rominit_data),
   .ROMINIT_VALID(rominit_valid)
   );

//////////////////////////////////////////////////////////////////////
// Connect joysticks and keyboard to HMI

hmi_t       hmi;

joykey joykey
  (
   .CLK_SYS(clk_sys),

   .JOYSTICK_0(joy0),
   .JOYSTICK_1(joy1),

   .PS2_KEY(ps2_key),

   .HMI(hmi)
   );

////////////////////////////////////////////////////////////////////
// Audio

// TODO: Add CDC for AUDIO_* into CLK_AUDIO

wire signed [8:0]   aud_pcm;
wire signed [15:0]  aud_pcm16, aud_out;

assign aud_pcm16 = {aud_pcm, 7'b0}; // sign-extend to 16 bits
assign aud_out = aud_pcm16 >>> 1;     // add some headroom

//assign AUDIO_S = '1; // signed
assign AUDIO_L = aud_out;
assign AUDIO_R = AUDIO_L;
//assign AUDIO_MIX = 0; // no mixing

///////////////////////   CLOCKS   ///////////////////////////////
wire clk_sys, locked;

pll pll(
    .inclk0(CLOCK_50),
    .c0(clk_sys),  // 28.63636200 MHz
    .locked(locked)
);

mapper_t mapper;
palette_t vdc_palette;
overscan_mask_t vdc_overscan_mask;

wire reset = ~locked | status[0] | buttons[1] | rominit_active;
assign mapper = mapper_t'(status[19:16]);
assign vdc_palette = palette_t'(status[8]);
assign vdc_overscan_mask = overscan_mask_t'(status[10:9]);

scv scv
  (
   .CLK(clk_sys),
   .RESB(~reset),

   .ROMINIT_SEL_BOOT(rominit_sel_boot),
   .ROMINIT_SEL_CHR(rominit_sel_chr),
   .ROMINIT_SEL_APU(rominit_sel_apu),
   .ROMINIT_SEL_CART(rominit_sel_cart),
   .ROMINIT_ADDR(rominit_addr),
   .ROMINIT_DATA(rominit_data),
   .ROMINIT_VALID(rominit_valid),

   .MAPPER(mapper),
   .VDC_PALETTE(vdc_palette),
   .VDC_OVERSCAN_MASK(vdc_overscan_mask),

   .HMI(hmi),

   .VID_PCE(CE_PIXEL),
   .VID_DE(VGA_DE),
   .VID_HS(video_hs),
   .VID_VS(video_vs),
   .VID_RGB({video_r, video_g, video_b}),

   .AUD_PCM(aud_pcm)
   );
	
// Data IO interface
wire        ioctl_download;
wire  [7:0] ioctl_index;
wire        ioctl_wr;
wire [24:0] ioctl_addr;
wire [7:0] ioctl_data;

data_io data_io(
    .clk_sys(clk_sys),
    .SPI_SCK(SPI_SCK),
    .SPI_SS2(SPI_SS2),
    .SPI_DI(SPI_DI),
    .clkref_n(1'b0),
    .ioctl_download(ioctl_download),
    .ioctl_index(ioctl_index),
    .ioctl_wr(ioctl_wr),
    .ioctl_addr(ioctl_addr),
    .ioctl_dout(ioctl_data)
);

// User IO interface
wire [31:0] joy0, joy1;
wire [10:0] ps2_key;
wire ps2_kbd_clk,ps2_kbd_data;
wire        key_pressed;
wire [7:0]  key_code;
wire        key_strobe;
wire        key_extended;

assign ps2_key = {key_strobe,key_pressed,key_extended,key_code}; 	
	
	
user_io #(
    .STRLEN($size(CONF_STR)>>3),
    .SD_IMAGES(2),
    .FEATURES(32'h0 | (BIG_OSD << 13))
) user_io(
    .clk_sys(clk_sys),
    .clk_sd(clk_sys),
    .conf_str(CONF_STR),
    .SPI_CLK(SPI_SCK),
    .SPI_SS_IO(CONF_DATA0),
    .SPI_MOSI(SPI_DI),
    .SPI_MISO(SPI_DO),
	 .ps2_kbd_clk      (ps2_kbd_clk),
	 .ps2_kbd_data     (ps2_kbd_data),
	 .key_strobe       (key_strobe),
	 .key_pressed      (key_pressed),
	 .key_extended     (key_extended),
	 .key_code         (key_code),
    .joystick_0(joy0),
    .joystick_1(joy1),
    .status(status)
);	
	
// Video signals
wire [7:0] video_r, video_g, video_b;
wire       video_hs, video_vs;
wire       video_hblank, video_vblank;
//wire       ce_pix;	
	
// Video output
mist_video #(
    .COLOR_DEPTH(8),
    .SD_HCNT_WIDTH(10),
//    .USE_BLANKS(1'b1),
    .OUT_COLOR_DEPTH(VGA_BITS),
    .BIG_OSD(BIG_OSD)
) mist_video(
    .clk_sys(clk_sys),
    .SPI_SCK(SPI_SCK),
    .SPI_SS3(SPI_SS3),
    .SPI_DI(SPI_DI),
    .R(video_r),
    .G(video_g),
    .B(video_b),
    .HSync(video_hs),
    .VSync(video_vs),
    .HBlank(video_hblank),
    .VBlank(video_vblank),
    .VGA_R(VGA_R),
    .VGA_G(VGA_G),
    .VGA_B(VGA_B),
    .VGA_VS(VGA_VS),
    .VGA_HS(VGA_HS),
    .ce_divider(1'b0),
    .scandoubler_disable(scandoubler_disable),
    .scanlines(status[8:6]),
    .ypbpr(ypbpr),
    .no_csync(no_csync)
);
	
//assign CLK_VIDEO = clk_sys;

assign LED = ps2_key;

`ifdef I2S_AUDIO
wire [31:0] clk_rate = 32'd28_000_000;

i2s i2s (
    .reset(reset),
    .clk(clk_sys),
    .clk_rate(clk_rate),

    .sclk(I2S_BCK),
    .lrclk(I2S_LRCK),
    .sdata(I2S_DATA),

    .left_chan(aud_out),
    .right_chan(aud_out)
);
`endif

endmodule
