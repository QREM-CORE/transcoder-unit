package transcoder_pkg;

    typedef enum logic [3:0] {
        TR_ROUTER_IDLE                  = 4'b0000,
        TR_ROUTER_MATH_TX               = 4'b0001,
        TR_ROUTER_MATH_TX_SNOOP         = 4'b0010,
        TR_ROUTER_MATH_RX               = 4'b0011,
        TR_ROUTER_MATH_RX_SNOOP         = 4'b0100,
        TR_ROUTER_BYPASS_TX             = 4'b0101,
        TR_ROUTER_BYPASS_RX             = 4'b0110,
        TR_ROUTER_MATH_RX_FROM_SEEDBANK = 4'b0111,
        TR_ROUTER_MATH_TX_TO_SEEDBANK   = 4'b1000
    } router_sel_t;

endpackage : transcoder_pkg
