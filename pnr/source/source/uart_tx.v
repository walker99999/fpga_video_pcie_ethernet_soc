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

module uart_tx #(
    parameter            BPS_NUM  =    16'd434
//  ���ò�����Ϊ4800ʱ��bitλ��ʱ�����ڸ���:50MHz set 10417  40MHz set 8333
//  ���ò�����Ϊ9600ʱ��bitλ��ʱ�����ڸ���:50MHz set 5208   40MHz set 4167
//  ���ò�����Ϊ115200ʱ��bitλ��ʱ�����ڸ���:50MHz set 434  40MHz set 347 12M set 104
)
(
    input          clk,         // clock                                   ʱ���ź�
    input [7:0]    tx_data,     // uart tx data signal byte��              �ȴ����͵��ֽ�����
    input          tx_pluse,    // uart tx enable signal,rising is active; ����ģ�鷢�ʹ����ź�
                   
    output reg     uart_tx,     // uart tx transmit data line              ����ģ�鴮�ڷ����ź���
    output         tx_busy      // uart tx module work states,high is busy;����ģ��æ״ָ̬ʾ
);

    //==========================================================================
    //wire and reg in the module
    //==========================================================================
    reg             tx_pluse_reg =0;
    
    reg	[2:0]	    tx_bit_cnt=0;	//the bits number has transmited.
                  
    reg	[2:0]	    tx_state=0;		//current state of tx state machine.
    reg	[2:0]	    tx_state_n=0;	//next state of tx state machine.
                  
    reg	[3:0]	    pluse_delay_cnt=0;
    reg             tx_en = 0;

    // uart tx state machine's state
    localparam  IDLE	   = 4'h0;	//tx state machine's state.����״̬
    localparam  SEND_START = 4'h1;	//tx state machine's state.����start״̬
    localparam  SEND_DATA  = 4'h2;	//tx state machine's state.��������״̬
    localparam  SEND_STOP  = 4'h3;	//tx state machine's state.����stop״̬
    localparam  SEND_END   = 4'h4;	//tx state machine's state.���ͽ���״̬
    
    // uart bps set  the clk's frequency is 50MHz
    reg	[15:0]	  clk_div_cnt=0;	//count for division the clock. 

    //==========================================================================
    //logic
    //==========================================================================
    assign tx_busy = (tx_state != IDLE); //һ��״̬ѭ������8bit����
    //some control single.
    
    always @(posedge clk)
    begin
        tx_pluse_reg <= `UD tx_pluse;
    end
    
    // uart ģ�鷢�͹���ʹ�ܱ�־�ź�  //���pulse�����ػ��en
    always @(posedge clk)
    begin
        if(~tx_pluse_reg & tx_pluse)
            tx_en <= 1'b1;
        else if(tx_state == SEND_END)
            tx_en <= 1'b0;
    end
    
    //division the clock to satisfy baud rate.�������ڼ�����
    always @ (posedge clk)
    begin
        if(clk_div_cnt == BPS_NUM || (~tx_pluse_reg & tx_pluse)) //�������� ���� ���ݴ���1bit�� ��������λ
            clk_div_cnt   <= `UD 16'h0;
        else
            clk_div_cnt   <= `UD clk_div_cnt + 16'h1;
    end
    
    //count the number has transmited.��������״̬�У�����bitλ�������Բ��������ۼ�
    always @ (posedge clk)
    begin
        if(!tx_en)
            tx_bit_cnt    <= `UD 3'h0;
        else if((tx_bit_cnt == 3'h7) && (clk_div_cnt == BPS_NUM))
            tx_bit_cnt    <= `UD 3'h0;
        else if((tx_state == SEND_DATA) && (clk_div_cnt == BPS_NUM))
            tx_bit_cnt    <= `UD tx_bit_cnt + 3'h1;
        else 
            tx_bit_cnt    <= `UD tx_bit_cnt;
    end
    
    //==========================================================================
    //transmitter state machine
    //==========================================================================
    
    //   state change ״̬��ת
    always @(posedge clk)
    begin
        tx_state <= tx_state_n;
    end
    
    //   state change condition ״̬��ת����������
    always @ (*)
    begin
      case(tx_state)
        IDLE   	:  
        begin
            if(~tx_pluse_reg & tx_pluse)   //����������16��ʱ��������ʱ����ת��������start״̬ //���������
    	        tx_state_n = SEND_START;
    	    else
    	        tx_state_n = tx_state;
    	end
        SEND_START	:  
        begin
            if(clk_div_cnt == BPS_NUM)               //����һ���������ڵĵ͵�ƽ����룬��������״̬
    	        tx_state_n = SEND_DATA;
    	    else
    		    tx_state_n = tx_state;
    	end
        SEND_DATA	:  
        begin
            if(tx_bit_cnt == 3'h7 && clk_div_cnt == BPS_NUM)    //��ʱ8���������ں󣨷�����8bit���ݣ�����ת������stop״̬
    	        tx_state_n = SEND_STOP;
    	    else
    		    tx_state_n = tx_state;
    	end
        SEND_STOP	:  
        begin
            if(clk_div_cnt == BPS_NUM)              //����ֹͣλ��Ϊ1���������ڣ���������һ���������ڵĸߵ�ƽ��֮����ת�����ͽ���״̬
    	        tx_state_n = SEND_END;
    	    else
    		    tx_state_n = tx_state;
        end
        SEND_END	:  tx_state_n = IDLE;
        default	:  tx_state_n = IDLE;
      endcase
    end
    
    //   logical ouput  ״̬�����
    always @ (posedge clk)
    begin
      if(tx_en)
      begin
          case(tx_state)
              IDLE       :  uart_tx  <= `UD 1'h1;           //����״̬����ߵ�ƽ
              SEND_START :  uart_tx  <= `UD 1'h0;           //start״̬����һ���������ڵĵ͵�ƽ
              SEND_DATA  :                                  //����״̬ÿ���������ڷ���һ��bit��
              begin
                  case(tx_bit_cnt)
                      3'h0  :  uart_tx  <= `UD tx_data[0];
                      3'h1  :  uart_tx  <= `UD tx_data[1];
                      3'h2  :  uart_tx  <= `UD tx_data[2];
                      3'h3  :  uart_tx  <= `UD tx_data[3];
                      3'h4  :  uart_tx  <= `UD tx_data[4];
                      3'h5  :  uart_tx  <= `UD tx_data[5];
                      3'h6  :  uart_tx  <= `UD tx_data[6];
                      3'h7  :  uart_tx  <= `UD tx_data[7];
                      default: uart_tx  <= `UD 1'h1;
                  endcase
              end
              SEND_STOP  :  uart_tx  <= `UD 1'h1;          //����ֹͣ״̬ ���1���������ڸߵ�ƽ
              default    :  uart_tx  <= `UD 1'h1;          // ����״̬Ĭ�������״̬һ�£����ָߵ�ƽ���
          endcase
      end
      else
          uart_tx <= `UD 1'h1;
    end
    
    endmodule
