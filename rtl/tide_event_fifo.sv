// -----------------------------------------------------------------------------
// tide_event_fifo.sv - reference 8-deep event FIFO in front of the temporal controller
// (doc 13 / doc 44 F13-F16).  tide_top (Agent 5) owns the real FIFO; this module is the
// reference the temporal controller was verified against, and the target of the
// FIFO properties in sva/formal_properties.sv.  See ASSUMPTIONS A2.
//
// Head is visible combinationally (valid_o/data_o); pop happens when pop_ready_i & valid_o.
// A push while full is REJECTED (event lost) and overrun_o pulses for one cycle.
// Packed event: {flags[4:0], z3, z2, z1, z0, t_arrival, t_event}  (197 bits)
// -----------------------------------------------------------------------------
module tide_event_fifo #(
    parameter integer DEPTH = 8,
    parameter integer WIDTH = 197
) (
    input  logic                       clk_i,
    input  logic                       rst_ni,
    input  logic                       push_valid_i,
    input  logic [WIDTH-1:0]           push_data_i,
    output logic                       push_accepted_o,
    input  logic                       pop_ready_i,
    output logic                       valid_o,
    output logic [WIDTH-1:0]           data_o,
    output logic                       full_o,
    output logic [$clog2(DEPTH):0]     occupancy_o,
    output logic                       overrun_o
);
    localparam integer AW = $clog2(DEPTH);
    logic [WIDTH-1:0] mem [0:DEPTH-1];
    logic [AW-1:0]    wr_ptr, rd_ptr;
    logic [AW:0]      count;

    assign full_o      = (count == DEPTH[AW:0]);
    assign valid_o     = (count != 0);
    assign data_o      = mem[rd_ptr];
    assign occupancy_o = count;

    wire do_push = push_valid_i & ~full_o;
    wire do_pop  = valid_o & pop_ready_i;
    assign push_accepted_o = do_push;

    always_ff @(posedge clk_i) begin
        if (!rst_ni) begin
            wr_ptr <= '0; rd_ptr <= '0; count <= '0; overrun_o <= 1'b0;
        end else begin
            overrun_o <= push_valid_i & full_o;
            if (do_push) begin
                mem[wr_ptr] <= push_data_i;
                wr_ptr      <= wr_ptr + 1'b1;
            end
            if (do_pop) rd_ptr <= rd_ptr + 1'b1;
            case ({do_push, do_pop})
                2'b10:   count <= count + 1'b1;
                2'b01:   count <= count - 1'b1;
                default: ;
            endcase
        end
    end
endmodule
