-- bally_rc_filter.vhd
-- Bally Astrocade output lowpass filter for Votrax SC01-A
--
-- Models the 4 cascaded RC lowpass stages on the Wizard of Wor PCB:
--   4 × (R=110kΩ, C=560pF)  →  fc ≈ 2584 Hz
--
-- Implemented as 4 cascaded 1st-order IIR sections:
--   y[n] = k·x[n] + (1-k)·y[n-1]
--
-- Coefficients (Q15, fs=48kHz):
--   k    = 9403   (= round(k * 32768))
--   1-k  = 23365  (= 32768 - k, exact complement)
--
-- All 4 stages are computed in a single clock cycle using VHDL variables.
-- Expects s_valid pulses at exactly 48 kHz (from sc01a_resamp output).
-- 18-bit version: all data paths widened from 16 to 18 bits.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity bally_rc_filter is
    port (
        clk         : in  std_logic;
        reset_n     : in  std_logic;
        s_in        : in  signed(17 downto 0);
        s_valid     : in  std_logic;
        s_out       : out signed(17 downto 0);
        s_out_valid : out std_logic
    );
end entity;

architecture rtl of bally_rc_filter is

    -- Q15 coefficients: k + (1-k) = 32768 exactly
    constant RC_K  : integer := 9403;          -- k   * 2^15
    constant RC_KM : integer := 32768 - RC_K;  -- (1-k) * 2^15 = 23365

    type state_t is array(0 to 3) of signed(17 downto 0);
    signal state : state_t := (others => (others => '0'));

    signal s_out_r       : signed(17 downto 0) := (others => '0');
    signal s_out_valid_r : std_logic := '0';

    -- One RC stage: y[n] = (RC_K*x + RC_KM*y_prev) >> 15
    -- Input range ±131071 → output guaranteed ±131071 (coefficients sum to 32768)
    function rc_stage(x : signed(17 downto 0); y_prev : signed(17 downto 0))
                      return signed is
        variable prod1 : signed(35 downto 0);
        variable prod2 : signed(35 downto 0);
        variable acc   : signed(36 downto 0);
    begin
        prod1 := x      * to_signed(RC_K,  18);
        prod2 := y_prev * to_signed(RC_KM, 18);
        acc   := resize(prod1, 37) + resize(prod2, 37);
        -- shift right 15, result fits in 18 bits
        return acc(32 downto 15);
    end function;

begin

    process(clk)
        variable v0, v1, v2, v3 : signed(17 downto 0);
    begin
        if rising_edge(clk) then
            if reset_n = '0' then
                state        <= (others => (others => '0'));
                s_out_r      <= (others => '0');
                s_out_valid_r <= '0';
            else
                s_out_valid_r <= s_valid;

                if s_valid = '1' then
                    -- 4 cascaded stages, computed serially within one cycle
                    -- each stage uses the OLD state(i) as feedback
                    v0 := rc_stage(s_in, state(0));
                    v1 := rc_stage(v0,   state(1));
                    v2 := rc_stage(v1,   state(2));
                    v3 := rc_stage(v2,   state(3));

                    state(0) <= v0;
                    state(1) <= v1;
                    state(2) <= v2;
                    state(3) <= v3;

                    s_out_r <= v3;
                end if;
            end if;
        end if;
    end process;

    s_out       <= s_out_r;
    s_out_valid <= s_out_valid_r;

end architecture;
