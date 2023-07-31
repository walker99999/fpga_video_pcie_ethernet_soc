`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:Meyesemi 
// Engineer: Will
// 
// Create Date: 2023-01-29 20:31  
// Design Name:  
// Module Name: 
// Project Name: 
// Target Devices: Pango
// Tool Versions: 
// Description: 
//      
// Dependencies: 
// 
// Revision:
// Revision 1.0 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

`define UD #1

module uart_rx # (
    parameter            BPS_NUM     =    16'd434
//  ���ò�����Ϊ4800ʱ��  bitλ��ʱ�����ڸ���:50MHz set 10417  40MHz set 8333
//  ���ò�����Ϊ9600ʱ��  bitλ��ʱ�����ڸ���:50MHz set 5208   40MHz set 4167
//  ���ò�����Ϊ115200ʱ��bitλ��ʱ�����ڸ���:50MHz set 434    40MHz set 347
)
(
      //input ports
      input             clk,
      input             uart_rx,
    
      //output ports
      output reg [7:0]  rx_data,
      output reg        rx_en,
      output            rx_finish,
      output     [15:0]   clk_div_cnt_wire
);

    // uart rx state machine's state
    localparam  IDLE         = 4'h0;    //����״̬���ȴ���ʼ�źŵ���.
    localparam  RECEIV_START = 4'h1;    //����Uart��ʼ�źţ��͵�ƽһ����������.
    localparam  RECEIV_DATA  = 4'h2;    //����Uart���������źţ��˹��̶��崫��8bit��ÿ�����������м�λ��ȡֵ��8�����ں���ת��stop״̬.
    localparam  RECEIV_STOP  = 4'h3;    //ֹͣ״̬�������Ǹߵ�ƽ�������״̬��һ�µİ���Э���׼��Ҫ�ȴ�һ��ֹͣλ��������״̬��ת.
    localparam  RECEIV_END   = 4'h4;    //������ת״̬.

    //==========================================================================
    //wire and reg in the module
    //==========================================================================
    reg    [2:0]        rx_state=0;       //current state of tx state machine. ��ǰ״̬
    reg    [2:0]        rx_state_n=0;     //next state of tx state machine.    ��һ��״̬
    reg    [7:0]        rx_data_reg;      //                                   �������ݻ���Ĵ���
    reg                 uart_rx_1d;       //save uart_rx one cycle.            ����uart_rxһ��ʱ������
    reg                 uart_rx_2d;       //save uart_rx one cycle.����uart_rx ǰ����ʱ������
    wire                start;            //active when start a byte receive.  ��⵽start�źű�־
    reg    [15:0]       clk_div_cnt;      //count for division the clock.      �������ڼ�����

    assign clk_div_cnt_wire = clk_div_cnt;

    //==========================================================================
    //logic
    //==========================================================================
    
    //some control single.  ��ʼ�������־
    always @ (posedge clk) 
    begin
         uart_rx_1d <= `UD uart_rx;
         uart_rx_2d <= `UD  uart_rx_1d;
    end

    assign start     = (!uart_rx) && (uart_rx_1d || uart_rx_2d);
    assign rx_finish = (rx_state == RECEIV_END);


    //division the clock to satisfy baud rate.�������ڼ�����   ȷ�������ʣ����ݴ�����
    always @ (posedge clk)
    begin
        if(rx_state == IDLE || clk_div_cnt == BPS_NUM)  //��λ
            clk_div_cnt   <= `UD 16'h0;
        else
            clk_div_cnt   <= `UD clk_div_cnt + 16'h1;
    end
    
    // receive bit data numbers 
    //�ڽ�������״̬�У����յ�bitλ������ÿһ���������ڼ�����1  //����ֽ�
    reg    [2:0]      rx_bit_cnt=0;    //the bits number has transmited.
    always @ (posedge clk)
    begin
        if(rx_state == IDLE)
            rx_bit_cnt <= `UD 3'h0;
        else if((rx_bit_cnt == 3'h7) && (clk_div_cnt == BPS_NUM))
            rx_bit_cnt <= `UD 3'h0;
        else if((rx_state == RECEIV_DATA) && (clk_div_cnt == BPS_NUM)) //�յ�����
            rx_bit_cnt <= `UD rx_bit_cnt + 3'h1;
        else 
            rx_bit_cnt <= `UD rx_bit_cnt;
    end

//==========================================================================
//receive state machine
//==========================================================================
    //״̬��״̬��ת
    always @(posedge clk)
    begin
        rx_state <= rx_state_n;
    end
    
    //״̬��״̬��ת��������ת����
    always @ (*)
    begin
      case(rx_state)
          IDLE       :  
          begin
              if(start)                                     //��⵽start�źŵ�������һ״̬��ת��start״̬
                  rx_state_n = RECEIV_START;
              else
                  rx_state_n = rx_state;
          end
          RECEIV_START    :  
          begin
              if(clk_div_cnt == BPS_NUM)                     //����ɽ���start��־�źţ�1bit start�ź�
                  rx_state_n = RECEIV_DATA;
              else
                  rx_state_n = rx_state;
          end
          RECEIV_DATA    :  
          begin
              if(rx_bit_cnt == 3'h7 && clk_div_cnt == BPS_NUM) //�����8bit���ݵĴ��䣬8bit�����ź�
                  rx_state_n = RECEIV_STOP;
              else
                  rx_state_n = rx_state;
          end
          RECEIV_STOP    :  
          begin
              if(clk_div_cnt == BPS_NUM)                       //����ɽ���stop��־�źţ�1bit stop
                  rx_state_n = RECEIV_END;
              else
                  rx_state_n = rx_state;
          end
          RECEIV_END    :  
          begin
              if(!uart_rx_1d)                                  //���������±����ͣ���ʾ�����ݴ����ַ���start��־�źţ���Ҫ��ת��start״̬
                  rx_state_n = RECEIV_START;
              else                                             //û������״������ʱ���ص�����״̬���ȴ�start�źŵĵ���
                  rx_state_n = IDLE;
          end
          default    :  rx_state_n = IDLE;
      endcase
    end
    
    // ״̬�����
    always @ (posedge clk)
    begin
        case(rx_state)
            IDLE         ,
            RECEIV_START :                               //�ڿ��к�start״̬ʱ���������ݻ���Ĵ���������ʹ����λ��
            begin
                rx_en <= `UD 1'b0;
                rx_data_reg <= `UD 8'h0;
            end
            RECEIV_DATA  :  
            begin
                if(clk_div_cnt == BPS_NUM[15:1])        //��һ���������ڵ��м�λ��ȡ�������ϴ�������ݣ�
                    rx_data_reg  <= `UD {uart_rx , rx_data_reg[7:1]};  //��ѭ�����Ƶķ�ʽ��uart_rx�������뻺��Ĵ��������λ��Uart�����λ��ǰ�����һ��bit�պ������λ��
            end
            RECEIV_STOP  : 
            begin
                rx_en   <= `UD 1'b1;                    // ���ʹ���źţ���ʾ���µ����������Ч
                rx_data <= `UD rx_data_reg;             // ������Ĵ�����ֵ��ֵ������Ĵ���
            end
            RECEIV_END    :  
            begin
                rx_data_reg <= `UD 8'h0;
                rx_en   <= `UD 1'b0;
            end
            default:    rx_en <= `UD 1'b0;
        endcase
    end

endmodule




