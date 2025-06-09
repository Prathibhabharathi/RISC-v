module cpu_tb;

    reg clk;
    integer i;

    // Instantiate CPU
    cpu uut (
        .clk(clk)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 10ns clock period
    end



    // Stimulus
    initial begin
        // Initialize registers manually
        uut.rf.regs[2] = 16'd10; // r2 = 10
        uut.rf.regs[3] = 16'd5;  // r3 = 5

        #100; // Run for 10 clock cycles

    end

endmodule
