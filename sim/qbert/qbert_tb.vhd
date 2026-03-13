-- qbert_tb.vhd
-- GHDL testbench for qbert (VotraxSound SC01 wrapper)
--
-- Replays events from votrax_tb_vectors and drives the SC01 interface directly:
--   EV_CLOCK    : freq (Hz) → clk_dac  via sc_hz = 950000 + (dac - 0xA0) * 5500
--   EV_PHONE    : data[5:0] = phoneme, data[7:6] = inflection
--   EV_INFLECTION: data[1:0] = inflection
--
-- Compile & run (see qbert_simulate.sh):
--   ghdl -a --std=08 ... qbert_tb.vhd
--   ghdl -e --std=08 qbert_tb
--   ghdl -r qbert_tb

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use std.env.stop;

use work.votrax_tb_vectors.all;

entity qbert_tb is
end entity;

architecture sim of qbert_tb is

    signal clk         : std_logic := '0';
    signal reset_n     : std_logic := '0';

    signal phoneme     : std_logic_vector(5 downto 0) := (others => '0');
    signal inflection  : std_logic_vector(1 downto 0) := "10";
    signal stb         : std_logic := '0';
    signal ar          : std_logic;
    signal clk_dac     : std_logic_vector(7 downto 0) := x"80";

    signal audio_out   : signed(15 downto 0);
    signal audio_valid : std_logic;

    constant CLK_PERIOD : time := 20 ns;  -- 50 MHz

    file audio_file : text;

begin

    -- ================================================================
    -- DUT
    -- ================================================================
    u_dut : entity work.qbert
        port map (
            clk         => clk,
            reset_n     => reset_n,
            phoneme     => phoneme,
            inflection  => inflection,
            stb         => stb,
            ar          => ar,
            clk_dac     => clk_dac,
            audio_out   => audio_out,
            audio_valid => audio_valid
        );

    -- ================================================================
    -- Clock
    -- ================================================================
    clk <= not clk after CLK_PERIOD / 2;

    -- ================================================================
    -- Stimulus
    -- ================================================================
    process
        variable phone      : integer;
        variable infl       : integer;
        variable freq       : integer;
        variable dac_i      : integer;
        variable prev_ts_us : integer := 0;
    begin
        file_open(audio_file, "audio_out.raw", write_mode);

        -- Reset
        reset_n <= '0';
        wait for 10 * CLK_PERIOD;
        reset_n <= '1';
        wait for 10 * CLK_PERIOD;

        -- Replay events from votrax_tb_vectors
        for i in 0 to N_EVENTS - 1 loop

            if INPUT_VECTORS(i).ts_us > prev_ts_us then
                wait for (INPUT_VECTORS(i).ts_us - prev_ts_us) * 1 us;
            end if;
            prev_ts_us := INPUT_VECTORS(i).ts_us;

            case INPUT_VECTORS(i).event is

                when EV_RESET =>
                    reset_n <= '0';
                    wait for 10 * CLK_PERIOD;
                    reset_n <= '1';

                when EV_CLOCK =>
                    -- Compute 8-bit DAC value from Hz: dac = (freq - 950000) / 5500 + 0xA0
                    freq  := INPUT_VECTORS(i).data;
                    dac_i := (freq - 950000) / 5500 + 16#A0#;
                    if dac_i < 16#40# then dac_i := 16#40#; end if;
                    if dac_i > 16#FF# then dac_i := 16#FF#; end if;
                    clk_dac <= std_logic_vector(to_unsigned(dac_i, 8));
                    report "Clock " & integer'image(freq) & " Hz -> clk_dac=0x"
                        & integer'image(dac_i) severity note;

                when EV_PHONE =>
                    phone := INPUT_VECTORS(i).data mod 64;
                    infl  := (INPUT_VECTORS(i).data / 64) mod 4;
                    phoneme   <= std_logic_vector(to_unsigned(phone, 6));
                    inflection <= std_logic_vector(to_unsigned(infl, 2));
                    report "Phone " & integer'image(i) & "/" & integer'image(N_EVENTS - 1)
                        & " : " & integer'image(phone)
                        & " infl=" & integer'image(infl)
                        & " @ " & time'image(now)
                        severity note;
                    wait for 1 * CLK_PERIOD;
                    stb <= '1';
                    wait for 2 * CLK_PERIOD;
                    stb <= '0';

                when EV_INFLECTION =>
                    inflection <= std_logic_vector(to_unsigned(INPUT_VECTORS(i).data mod 4, 2));

                when others => null;

            end case;

        end loop;

        -- Wait for last phoneme to finish
        wait for 500 ms;

        file_close(audio_file);
        report "Done! Samples written to audio_out.raw" severity note;
        report "Convert with: python3 ../../tools/raw_to_wave.py audio_out.raw audio_out.wav" severity note;
        stop;
    end process;

    -- ================================================================
    -- Audio capture: write one sample per audio_valid pulse
    -- ================================================================
    process (clk)
        variable l      : line;
        variable sample : integer;
    begin
        if rising_edge(clk) then
            if reset_n = '1' and audio_valid = '1' then
                sample := to_integer(audio_out);
                write(l, integer'image(sample));
                writeline(audio_file, l);
            end if;
        end if;
    end process;

end architecture;
