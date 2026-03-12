-- bally_tb.vhd
-- GHDL testbench for bally (VotraxSound wrapper)
--
-- Replays events from votrax_tb_vectors and maps them to the
-- Bally I_VOTRAX_DATA byte format:
--   bit 7 : infl_sel  (1 = inflection "00", 0 = inflection "10")
--   bit 6 : clk_sel   (0 = 0x7D ≈ 757 kHz, 1 = 0x82 ≈ 785 kHz)
--   bits 5:0 : phoneme
--
-- EV_CLOCK mapping: freq >= 770000 Hz → clk_sel='1', else '0'
-- EV_PHONE inflection mapping: infl=0 → infl_sel='1', else '0'
--
-- Compile & run (see bally_simulate.sh):
--   ghdl -a --std=08 ... bally_tb.vhd
--   ghdl -e --std=08 bally_tb
--   ghdl -r bally_tb --wave=wave.ghw

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use std.env.stop;

use work.votrax_tb_vectors.all;

entity bally_tb is
    generic (
        SKIP_START : integer := 3;
        SKIP_END   : integer := 29
    );
end entity;

architecture sim of bally_tb is

    signal clk         : std_logic := '0';
    signal reset_n     : std_logic := '0';

    -- Bally interface
    signal votrax_data : std_logic_vector(7 downto 0) := (others => '0');
    signal votrax_stb  : std_logic := '0';
    signal votrax_ar   : std_logic;

    signal audio_out_l : std_logic_vector(15 downto 0);
    signal audio_out_r : std_logic_vector(15 downto 0);
    signal audio_valid : std_logic;

    -- Persistent sub-fields (combined into votrax_data)
    signal phoneme_r   : std_logic_vector(5 downto 0) := (others => '0');
    signal clk_sel_r   : std_logic := '0';   -- 0=757kHz, 1=785kHz
    signal infl_sel_r  : std_logic := '0';   -- 0=infl"10", 1=infl"00"

    constant CLK_PERIOD : time := 35 ns;  -- 28 MHz

    file audio_file : text;

begin

    -- Reconstruct I_VOTRAX_DATA from sub-fields
    votrax_data <= infl_sel_r & clk_sel_r & phoneme_r;

    -- ================================================================
    -- DUT
    -- ================================================================
    u_dut : entity work.bally
        port map (
            CLK           => clk,
            I_RESET_L     => reset_n,
            I_VOTRAX_DATA => votrax_data,
            I_VOTRAX_STB  => votrax_stb,
            O_VOTRAX_AR   => votrax_ar,
            s_enable      => '1',
            audio_out_l   => audio_out_l,
            audio_out_r   => audio_out_r,
            audio_valid   => audio_valid
        );

    -- ================================================================
    -- Clock
    -- ================================================================
    clk <= not clk after CLK_PERIOD / 2;

    -- ================================================================
    -- Stimulus
    -- ================================================================
    process
        variable phone   : integer;
        variable infl    : integer;
        variable prev_ts_us : integer := 0;
        variable skipped : boolean := false;
        variable freq    : integer;
    begin
        file_open(audio_file, "audio_out.raw", write_mode);

        -- Reset
        reset_n <= '0';
        wait for 10 * CLK_PERIOD;
        reset_n <= '1';
        wait for 10 * CLK_PERIOD;

        -- Replay events from votrax_tb_vectors
        for i in 0 to N_EVENTS - 1 loop

            if i < SKIP_START or i >= SKIP_END or true then
                if INPUT_VECTORS(i).ts_us > prev_ts_us and not skipped then
                    wait for (INPUT_VECTORS(i).ts_us - prev_ts_us) * 1 us;
                end if;
                skipped := false;
                prev_ts_us := INPUT_VECTORS(i).ts_us;

                case INPUT_VECTORS(i).event is

                    when EV_RESET =>
                        reset_n <= '0';
                        wait for 10 * CLK_PERIOD;
                        reset_n <= '1';

                    when EV_CLOCK =>
                        -- Map Hz frequency to the two Bally-supported clock rates
                        freq := INPUT_VECTORS(i).data;
                        if freq >= 770000 then
                            clk_sel_r <= '1';   -- 0x82 ≈ 785 kHz
                        else
                            clk_sel_r <= '0';   -- 0x7D ≈ 757 kHz
                        end if;
                        report "Clock " & integer'image(freq) & " Hz -> clk_sel="
                            & integer'image(0) severity note;

                    when EV_PHONE =>
                        phone := INPUT_VECTORS(i).data mod 64;
                        infl  := (INPUT_VECTORS(i).data / 64) mod 4;
                        -- Map 2-bit inflection to 1-bit infl_sel:
                        --   infl=0 → "00" → infl_sel='1'
                        --   infl≠0 → "10" → infl_sel='0'
                        if infl = 0 then
                            infl_sel_r <= '1';
                        else
                            infl_sel_r <= '0';
                        end if;
                        phoneme_r <= std_logic_vector(to_unsigned(phone, 6));
                        report "Phone " & integer'image(i) & "/" & integer'image(N_EVENTS - 1)
                            & " : " & integer'image(phone)
                            & " infl=" & integer'image(infl)
                            & " @ " & time'image(now)
                            severity note;
                        wait for 1 * CLK_PERIOD;   -- let signals settle
                        votrax_stb <= '1';
                        wait for 2 * CLK_PERIOD;
                        votrax_stb <= '0';

                    when EV_INFLECTION =>
                        if INPUT_VECTORS(i).data = 0 then
                            infl_sel_r <= '1';
                        else
                            infl_sel_r <= '0';
                        end if;

                    when others => null;

                end case;
            else
                report "skipped: " & integer'image(i) severity note;
                prev_ts_us := INPUT_VECTORS(i).ts_us;
                skipped := true;
            end if;

        end loop;

        -- Wait for last phoneme to finish
        wait for 500 ms;

        file_close(audio_file);
        report "Done! Samples written to audio_out.raw" severity note;
        report "Convert with: python3 raw_to_wav.py audio_out.raw audio_out.wav" severity note;
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
                sample := to_integer(signed(audio_out_l));
                write(l, integer'image(sample));
                writeline(audio_file, l);
            end if;
        end if;
    end process;

end architecture;
