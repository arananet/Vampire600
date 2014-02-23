-----------------------------------------------------------------------------------------
-----------------------------------------------------------------------------------------
-- Filename: Vampire_600.vhd                                                           --
-- Copyright(c): Majsta, AMR - 2011-2013                                               --    
-- Contact info:  www.majsta.com                                                       --
-- Publishing date:     November 1, 2013                                               --
-- Licensed: GNU General Public License v3 (GPL-3)                                     --
--           For more informations about license visit: http://www.gnu.org/licenses/   --            
-- Abstract: This is the Top Level Design file for Amiga Vampire 600 accelerator       --
-- Core version: 0.1                                                                   --
-- Core base: TG68.C                                                                   --
-- Emulated CPU: MC68000                                                               --
-- Operational frequency: 87.5MHz                                                      --
-- Cache: No                                                                           --
-- Mips: 2.75                                                                          --
-- Dhrystones: 2640                                                                    --
-- Chip Speed vs A600: 1.86                                                            --
-- Disk speed in BYTES/SEC: 908,120                                                    --
-- Autoconfig Mem: 5MB                                                                 --
-- Memory space: $C00000 - $CFFFFF, $200000 - $5FFFFF                                  --
-- Total FastRam: 5MB                                                                  --
-- PCMCIA friendly: Yes                                                                --
-----------------------------------------------------------------------------------------
-----------------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.std_logic_unsigned.all;
use ieee.numeric_std.all;

entity Vampire_600 is
   port(
         iSYS_CLK   : in std_logic;    -- 50MHz clock
         reset_a    : in std_logic;    -- System Reset, input from Amiga motherboard
         reset_b    : out std_logic;   -- System Reset, output to Amiga motherboard
         halt_b     : out std_logic;   -- System Halt, output to Amiga motherboard
         clk_7Mhz   : in std_logic;    -- 7MHz clock, input from Amiga 600 motherboard             
--
-- MC68000 signals
--
        iTG68_IPLn    : in std_logic_vector(2 downto 0);
        iTG68_DTACKn  : in std_logic;
        oTG68_ADDR    : out std_logic_vector(23 downto 1);
        oTG68_ASn     : out std_logic;
        iASn          : in std_logic;
        oTG68_UDSn    : out std_logic;
        oTG68_LDSn    : out std_logic;
        oTG68_RW      : out std_logic;        
        ioTG68_DATA   : inout std_logic_vector(15 downto 0);          
        oBRn          : out std_logic;
        iBGn          : in std_logic;
        oBGACKn       : out std_logic;
--      FC            : out std_logic_vector(2 downto 0); -- Not used|
--      iBERRn        : in std_logic;                     --         |
--      iVPA          : in std_logic;                     --         |
--      oVMA          : in std_logic;                     --         |
--      oE            : out std_logic;                    ------------
 
        LED : out std_logic; 

--        
-- Voltage level translation description            
--
-- Devices ALVT or ALVC - 16 bits - 8 bits X 2 banks
-- DIR <= '1'; -- From FPGA to Amiga bus
-- DIR <= '0'; -- From Amiga bus to FPGA 
-- OE  <= '0'; -- Enable device
-- OE  <= '1'; -- Hi-Z

--
-- ALVT U1 (U1_1DIR_C controls signals: IPLn(2 downto 0), reset_a, DTACKn, clk_7Mhz, iBGn, iASn)
--         (U1_2DIR_C controls signals: oTG68_RW, oTG68_LDSn, oTG68_UDSn, oTG68_ASn)            
--
         U1_1DIR_C    : out std_logic:='0'; -- From Amiga bus to FPGA
         U1_1OE_C     : out std_logic:='0'; -- Allways enabled
            
         U1_2DIR_C    : out std_logic;      
         U1_2OE_C     : out std_logic:='1'; -- Hi-Z for now

--
-- ALVT U2 (U2_1DIR_C, U2_2DIR_C constrols signals: ioTG68_DATA(15 downto 0)            
--
         U2_1DIR_C    : out std_logic:='0'; -- From Amiga bus to FPGA
         U2_1OE_C     : out std_logic:='0'; -- Allways enabled            
         U2_2DIR_C    : out std_logic:='0'; -- From Amiga bus to FPGA
         U2_2OE_C     : out std_logic:='0'; -- Allways enabled
--
-- ALVT U3 (U3_1DIR_C controls signals: oBRn, oBGACKn, reset_b, halt_b) 
--              (U3_2DIR_C controls signals: oTG68_ADDR(23 downto 17)
--
         U3_1DIR_C    : out std_logic:='1'; -- From FPGA to Amiga bus 
         U3_1OE_C     : out std_logic:='0'; -- Allways enabled
            
         U3_2DIR_C    : out std_logic;       
         U3_2OE_C     : out std_logic:='1'; -- Hi-Z for now
--
-- ALVT U4 (U4_1DIR_C, U4_2DIR_C controls signals: oTG68_ADDR(16 downto 1)        
--
         U4_1DIR_C    : out std_logic;      
         U4_1OE_C     : out std_logic:='1'; -- Hi-Z for now
         U4_2DIR_C    : out std_logic;      
         U4_2OE_C     : out std_logic:='1'; -- Hi-Z for now

--            
-- SDRAM signals
--
         sdr_addr : out std_logic_vector(12 downto 0);   -- SDRAM address
         sdr_data : inout std_logic_vector(15 downto 0); -- SDRAM data
         sdr_ba   : out std_logic_vector(1 downto 0);    -- Bank select
         sdr_clk  : out std_logic;                       -- SDRAM clock.  Should be same speed as sysclk, but phase shifted by roughly -1.5ns
         sdr_cke  : out std_logic;                       -- We just hold this high
         sdr_dqm  : out std_logic_vector(1 downto 0);    -- Data mask
         sdr_cs   : out std_logic;                       -- Chip select, active low
         sdr_we   : out std_logic;                       -- Write enable, active low
         sdr_cas  : out std_logic;                       -- CAS, active low
         sdr_ras  : out std_logic                        -- RAS, active low    
 );
end Vampire_600;

ARCHITECTURE logic OF Vampire_600 IS
--
-- Clock signals
-- 
signal sysclk     : std_logic;                           -- Master clock, 87.5MHz
signal amigaclk_r : std_logic;                           -- Amiga 7MHz clock, synchronised to sysclk
signal amigaclk   : std_logic;                           -- Amiga 7MHz clock, double-synchronised to sysclk.
signal amigaclk_phase1 : std_logic_vector(4 downto 0);   -- Count sysclks since the falling edge of 7MHz
signal amigaclk_phase2 : std_logic_vector(4 downto 0);   -- Count sysclks since the rising edge of 7MHz
signal amiga_risingedge   : std_logic;                   -- Rising edge of 7MHz clock
signal amiga_fallingedge  : std_logic;                   -- Falling edge of 7MHz clock

-- FSM
signal TG68_RESETn : std_logic;                          -- TG68 reset, 
signal fsm_ena     : std_logic := '0';

type mystates is
    (init,reset,state1,state2,main,delay1,delay2,delay3,delay4,
    writeS0,writeS1,writeS2,writeS3,writeS4,writeS5,writeS6,writeS7,writeS8,
    readS0,readS1,readS2,readS3,readS4,readS5,readS6,readS7,
    fast_access,fast_autoconfig);

signal mystate : mystates :=init;                        -- Declare state machine variable with initial value of "init"

signal amiga_addr : std_logic_vector(23 downto 0);       -- CPU's current address
signal reset_counter : unsigned(15 downto 0) := X"FFFF";

-- CPU signals
signal cpu_datain  : std_logic_vector(15 downto 0);      -- Data provided by us to CPU
signal cpu_dataout : std_logic_vector(15 downto 0);      -- Data received from the CPU
signal cpu_addr    : std_logic_vector(31 downto 1);      -- CPU's current address
signal FC          : std_logic_vector(2 downto 0);
signal cpu_as      : std_logic;                          -- Address strobe
signal cpu_uds     : std_logic;                          -- Upper data strobe
signal cpu_lds     : std_logic;                          -- Lower data strobe
signal cpu_r_w     : std_logic;                          -- Read(high), Write(low)
signal cpustate    : std_logic_vector(1 downto 0);
signal cpu_clkena  : std_logic :='0';
signal tg68_ready  : std_logic;                          -- High when the CPU is initialised and ready for use.

signal sel_fastram : std_logic;
signal cpu_ipl_s   : std_logic_vector(2 downto 0);
signal cpu_ipl     : std_logic_vector(2 downto 0);
signal nullsig0    : std_logic;
signal DTACK_s     : std_logic;
signal DTACK       : std_logic;

-- SDRAM signals
signal sdr_ready       : std_logic;
signal sdr_cpuena      : std_logic;
signal sdr_req_pending : std_logic;
signal sdr_datatocpu   : std_logic_vector(15 downto 0);

signal sampled_reset   : std_logic;
signal sampled_reset_s : std_logic;

-- Fast RAM signals
signal sdram_addr    : std_logic_vector(31 downto 1);    -- CPU's current address
signal sel_24bit     : std_logic;
signal sel_slowram   : std_logic;

-- Autoconfig signals
signal sel_autoconfig   : std_logic;
signal autoconfig_data  : std_logic_vector(3 downto 0);  -- 24-bit (Zorro II) autoconfig data;
signal autoconfig_out   : std_logic_vector(1 downto 0);  -- Select between 24- and 32-bit autoconfig.

BEGIN
--
-- PLL for generation 87.5MHz Master clock from 50MHz
--
mySysClock : entity work.SysClk
   port map(
      inclk0 => iSYS_CLK,
      c0 => sysclk,        -- System clock, 87.5MHz  
      c1 => sdr_clk        -- SDRAM clock, same as sysclk but phase shifted for -1.00ns.
   );
    
--
-- FSM that controls the resetting of the TG68 core and the bus arbitration between the TG68 and the MC68k CPU.
--    
myreset_fsm : entity work.reset_fsm
   port map( 
      iCLK_7MHZ    => clk_7Mhz,      -- 7MHz clock from Amiga motherboard
      iRESETn      => reset_a,       -- Active low reset from Amiga motherboard
      fsm_ena      => fsm_ena,       -- Enable signal, active high
-- MC68k signals
      iASn         => iASn,          -- MC68k /AS, input from MC68k
      iDTACKn      => iTG68_DTACKn,  -- MC68k /DTACK
      ioBRn        => oBRn,          -- Bus Request output to MC68k
      iBGn         => iBGn,          -- Bus Grant input from MC68k
      oBGACKn      => oBGACKn,       -- Bus Grant Acknowledge to the MC68k
-- TG68 signals
      oTG68_RESETn => TG68_RESETn    -- TG68 reset
   );        
--
-- TG68.C core
--
myTG68 : entity work.TG68KdotC_Kernel
   generic map
   (
      SR_Read        =>    0,
      VBR_Stackframe =>    0,
      extAddr_Mode   => 0,
      MUL_Mode       => 0,
      DIV_Mode       => 0,
      BitField       => 0
    )
   port map
   (
      clk         => sysclk,
      nReset      => TG68_RESETn,   -- Contributes to reset, activated after bus arbitration sequence is done
      clkena_in   => cpu_clkena,        
      data_in     => cpu_datain,
      IPL         => cpu_ipl, 
      IPL_autovector => '0',
      CPU         => "00",          -- MC68000 emulation mode
      addr(31 downto 1) => cpu_addr,
      addr(0)     => nullsig0,
      data_write  => cpu_dataout,
      nWr         => cpu_r_w,
      nUDS        => cpu_uds,
      nLDS        => cpu_lds,
      busstate    => cpustate,
      nResetOut   => tg68_ready,
      FC          => FC,
-- for debug        
      skipFetch   => open,
      regin       => open
    );
--
-- SDRAM controller
--
sdr_cke<='1';

mysdram : entity work.sdram 
   generic map
   (
      rows => 13,
      cols => 10
   )
   port map
   (
-- Physical connections to the SDRAM
      sdata  => sdr_data,
      sdaddr => sdr_addr,
      sd_we  => sdr_we,
      sd_ras => sdr_ras,
      sd_cas => sdr_cas,
      sd_cs  => sdr_cs,
      dqm    => sdr_dqm,
      ba     => sdr_ba,
-- Housekeeping
      sysclk    => sysclk,
      reset     => TG68_RESETn,
      reset_out => sdr_ready,
      Addr1     => sdram_addr & '0',   -- Addresses are unchanged, so $200000-$9fffff are passed through directly
      req1      => sdr_req_pending,
      wr1       => cpu_r_w,
      wrL1      => cpu_lds, 
      wrU1      => cpu_uds, 
      datawr1   => cpu_dataout,
      dataout1  => sdr_datatocpu,
      dtack1    => sdr_cpuena
   );    

-- Detection of rising and fallig edges of Amiga clock
process(sysclk,clk_7Mhz)
begin
    if rising_edge(sysclk) then
      -- Clean up the clock signal
        amigaclk<=amigaclk_r;                     -- Usable Synchronised Amiga clock
        amigaclk_r<=clk_7Mhz;                     -- First stage synchronised Amiga clock.
        
        if amigaclk='1' then
            amigaclk_phase1<="00000";             -- Reset counter while clock is high.
            amigaclk_phase2<=amigaclk_phase2+1;   -- Count sysclks since the rising edge
        else
            amigaclk_phase2<="00000";             -- Reset counter while clock is low
            amigaclk_phase1<=amigaclk_phase1+1;   -- Count sysclks since the falling edge
        end if;
        
      -- An enable signal we can use instead of rising_edge() or falling_edge()
            amiga_risingedge<='0';
        if amigaclk_phase1="00011" then           -- phase 4 - adjusted to work with 87.5MHz clock generated by PLL
            amiga_risingedge<='1';
        end if;    

            amiga_fallingedge<='0';
        if amigaclk_phase2="00011" then           -- phase 4 - adjusted to work with 87.5MHz clock generated by PLL
            amiga_fallingedge<='1';
        end if;    
            
    end if;
    
end process;

oTG68_ADDR <= amiga_addr(23 downto 1);

-- Synchronise the sampled_reset and IPL signals
process(sysclk,sampled_reset_s)
begin
    if rising_edge(sysclk) then
        sampled_reset_s<=reset_a;
        sampled_reset<=sampled_reset_s;
        -- Interrupt signals - we sync once on the sysclk edge, then again on the 7MHz rising edge.
        -- This should ensure that the TG68 doesn't sample IPL during transition.
        cpu_ipl_s<=iTG68_IPLn;
        if amiga_risingedge = '1' then
            cpu_ipl<=cpu_ipl_s;
        end if;
    end if;
end process;

-- Double-sync DTACK signal
process(sysclk)
begin
    if rising_edge(sysclk) then
        DTACK_s<=iTG68_DTACKn;
        DTACK<=DTACK_s;
    end if;
end process;

-- Address decoding...
sel_24bit <= '1' when
    cpu_addr(31 downto 24)=X"00" else '0'; -- 0x000000 to 0xffffff

-- The first set of autoconfig data (Zorro II RAM)
sel_autoconfig <= '1' when
    sel_24bit='1' and autoconfig_out="01" and cpu_addr(23 downto 16)=X"E8" else '0'; -- Autoconfig space

sel_slowram <= '1' when sel_24bit='1' and cpu_addr(23 downto 20)=X"C" else '0';

sel_fastram <= '1' when
    cpu_addr(31 downto 21)="000"&X"01" or    -- 0x200000 - 0x3fffff
    cpu_addr(31 downto 21)="000"&X"02"       -- 0x400000 - 0x5fffff
--    cpu_addr(31 downto 21)="000"&X"03" or  -- 0x600000 - 0x7fffff
--    cpu_addr(31 downto 21)="000"&X"04"     -- 0x800000 - 0x9fffff
        else '0';

sdram_addr<=cpu_addr(31 downto 1);
  
-- FastRam autoconfig mechanism -- Zorro II RAM (Up to 8 meg at 0x200000)
process(sysclk)
begin     
    autoconfig_data <= "1111";
    CASE cpu_addr(6 downto 1) IS
        WHEN "000000" => autoconfig_data <= "1110";        -- Zorro-II card, add mem, no ROM
        WHEN "000001" => autoconfig_data <= "0111";        -- 0111=4MB -- 0000=8MB  
        WHEN "001000" => autoconfig_data <= "1110";        
        WHEN "001001" => autoconfig_data <= "1100";        
        WHEN "001010" => autoconfig_data <= "0110";        -- Manufacturer ID, 5016 = Majsta
        WHEN "001011" => autoconfig_data <= "0111";        
        WHEN "010011" => autoconfig_data <= "1110";        -- Serial=1
        WHEN OTHERS => null;
    END CASE;
end process;


process(sysclk,reset_counter,sampled_reset)
begin

  if rising_edge(sysclk) then
      cpu_clkena<='0';                        -- The TG68.C will be paused by default, delay2 state will allow it to run for 1 cycle.    
         case mystate is    
            when init =>
                ioTG68_DATA <= (others=>'Z'); -- Make the data lines high-impedence, suitable for input
                reset_counter<=X"FFFF";          
                reset_b <= '0';               -- Reset the Amiga|
                halt_b <= '0';                -------------------
                sdr_req_pending<='0';
                mystate<=reset;
                
            when reset =>
                autoconfig_out<="01";
                if amiga_risingedge = '1' then
                    if reset_counter=X"0000" then                                                 
                        reset_b <= '1';                -- Release the reset signal|
                        halt_b <= '1';                 ----------------------------
                        mystate<=state1;
                    end if;
                    reset_counter<=reset_counter-1;
                end if;

            when state1 =>                                              
                reset_b <= '1';
                halt_b <= '1';             
                fsm_ena <= '1';                         -- Enables reset_fsm - Bus arbitration sequence
                mystate<=state2;
        
            when state2 =>
                if amiga_risingedge = '1' and fsm_ena = '1' and  TG68_RESETn = '1' then     -- Check that we own the Bus        
                        
--            U1_1DIR_C      <= '0';  -- Signals: IPLn(2 downto 0), reset_a, DTACKn, clk_7Mhz, iBGn, iASn|
--            U1_1OE_C       <= '0';  -- Enabled as input by default--------------------------------------

            U1_2DIR_C      <= '1';    -- Signals: oTG68_RW, oTG68_LDSn, oTG68_UDSn, oTG68_ASn |
            U1_2OE_C       <= '0';    -- as output from FPGA to Amiga bus----------------------

            U2_1DIR_C      <= '0';    -- Signals: ioTG68_DATA(15 downto 0) as input|
            U2_1OE_C       <= '0';    -- from Amiga bus to FPGA                    |
            U2_2DIR_C      <= '0';    --                                           |
            U2_2OE_C       <= '0';    ----------------------------------------------                                                            
                    
--            U3_1DIR_C      <= '1';  -- Signals: oBRn, oBGACKn, reset_b, halt_b|
--            U3_1OE_C       <= '0';  -- Enabled as output by default------------

            U3_2DIR_C      <= '1';    -- Signals: oTG68_ADDR(23 downto 1) as output| 
            U3_2OE_C       <= '0';    -- from FPGA to Amiga bus                    |
                                      --                                           |
            U4_1DIR_C      <= '1';    --                                           |
            U4_1OE_C       <= '0';    --                                           |            
            U4_2DIR_C      <= '1';    --                                           |
            U4_2OE_C       <= '0';    ----------------------------------------------
                                                                                        
            oTG68_ASn      <= '1';    -- Drive control signals active High|
            oTG68_RW       <= '1';    --                                  |
            oTG68_UDSn     <= '1';    --                                  |
            oTG68_LDSn     <= '1';    -------------------------------------
            mystate<=main;                    
            end if;
            
            when main =>
                if cpustate    =    "01" then                           -- No memaccess                        
                        mystate<=delay1;                        
                    elsif sel_slowram  = '1' or sel_fastram = '1' then  -- Slowram or FastRam detection                            
                                sdr_req_pending <= '1';                        
                                mystate    <= fast_access;                                                        
                    elsif sel_autoconfig = '1' then                     -- Activation of autoconfig mechanism -- Zorro II RAM 
                                mystate    <=    fast_autoconfig;                                                        
                    else                                                -- Normal bus cycle
                                                        
                        if cpu_r_w='0' then                             -- Write cycle.
                            mystate <= writeS0;                    
                        else                                            -- Read cycle        
                            mystate <= readS0;
                        end if;                                            
                end if;
            
            if sampled_reset='0' then            
                mystate <= init;            
            end if;
            
            when fast_access =>
                    cpu_datain<=sdr_datatocpu;       -- Copy data from SDRAM to CPU
                if sdr_cpuena='0' then
                    sdr_req_pending<='0';            -- Cancel the request, since it's been acknowledged.                                    
                    if cpu_r_w='0' then              -- Writing cycle, let the CPU continue
                        cpu_clkena<='1';             -- Alow TG68 to run one clock    
                        mystate<=delay3;
                    else
                        mystate<=delay1;             -- Reading settle data 
                    end if;
                end if;                
                
            when fast_autoconfig =>            
                if sel_autoconfig = '1' then
                    cpu_datain <=autoconfig_data & sdr_datatocpu(11 downto 0);                
                    if cpu_r_w='0' and cpu_addr(6 downto 1)="100100" then     -- Register 0x48 - config
                        autoconfig_out<="00";
                    end if;
                    mystate<=delay1;                            
                end if;                                                
                
            when delay1 =>
                    mystate<=delay2;
                
            when delay2 =>
                    cpu_clkena<='1';                  -- Alow TG68 to run one clock            
                    mystate<=delay3;
        
                
            when delay3 =>
                    mystate<=delay4;
                
            when delay4 =>
                    mystate<=main;
    
-- **** WRITE CYCLE ****
            when writeS0 =>
--                if amiga_risingedge = '1' then      -- Rising edge of S0
                    oTG68_RW        <='1';            -- Drive /RW high                                            
                    mystate<=writeS1;                    
--                end if;

            when writeS1 =>
                if amiga_fallingedge = '1' then              -- Falling edge of S1 
                    U3_2OE_C        <=    '0';               -- Enable address bus|                
                    U4_1OE_C        <=    '0';               --                   | 
                    U4_2OE_C        <=    '0';               ----------------------
                    amiga_addr <= cpu_addr(23 downto 1)&'0'; -- Drive a valid address on the address bus
                    mystate<=writeS2;
                end if;
                
            when writeS2 =>    
                if amiga_risingedge = '1' then         -- Rising edge of S2
                    oTG68_ASn    <=    '0';            -- Pull /AS low to indicate that a valid address is on the bus
                    oTG68_RW        <=    '0';         -- Pull /RW low to indicate write cycle
                    mystate<=writeS3;
                end if;
                
            when writeS3 =>
                if amiga_fallingedge = '1' then        -- Falling edge of S3        
                    U2_1DIR_C     <= '1';              -- Data Bus as output from FPGA|
                    U2_1OE_C      <= '0';              --                             |
                    U2_2DIR_C     <= '1';              --                             |
                    U2_2OE_C      <= '0';              --------------------------------
                    ioTG68_DATA<=cpu_dataout;          -- Write Data 
                    mystate<=writeS4;
                end if;

            when writeS4 =>
                if amiga_risingedge = '1' then        -- Rising edge of S4
                    oTG68_UDSn  <=cpu_uds;            -- Transfer UDS/LDS state to Amiga bus|
                    oTG68_LDSn  <=cpu_lds;            ---------------------------------------
                    if DTACK ='0' then                -- Wait for cycle termination signal (DTACK) and inserts wait states
                        mystate<=writeS5;    
                    end if;                    
                end if;
                
            when writeS5 =>     
              if amiga_fallingedge = '1' then         -- Falling edge of S5        
                  mystate<=writeS6;
                end if;
                              
            when writeS6 =>     
                if amiga_risingedge = '1' then        -- Rising edge of S6     
                    mystate<=writeS7;
                end if;
                
            when writeS7 =>        
                if amiga_fallingedge = '1' then       -- Falling edge of S7        
                    oTG68_ASn  <= '1';                -- Negate /AS,/USD,/LDS|
                    oTG68_UDSn <= '1';                --                     |
                    oTG68_LDSn <= '1';                ------------------------                
                    mystate<=writeS8;
                end if;

            when writeS8 =>                           -- This is just cleaning up in preparation for the next cycle                                                            
                if amiga_risingedge = '1' then        -- The rising edge here is the rising edge of the next cycle's S0 state            
                    oTG68_RW    <='1';                -- Drive /RW high
                    U2_1DIR_C <= '0';                 -- Data Bus as input to FPGA|
                    U2_1OE_C  <= '0';                 --                          |
                    U2_2DIR_C <= '0';                 --                          |
                    U2_2OE_C  <= '0';                 -----------------------------
                    ioTG68_DATA <= (others=>'Z');     -- Place Data bus in Hi-Z
                    
                    U3_2OE_C     <= '1';              -- Disable address bus|                                    
                    U4_1OE_C     <= '1';              --                    |
                    U4_2OE_C     <= '1';              -----------------------    
                    amiga_addr  <= (others=>'Z');     -- Place Address bus in Hi-Z
                    mystate<=delay1;                        
                end if;        
                    
-- **** READ CYCLE ****
            when readS0 =>    
--                if amiga_risingedge = '1' then      -- Rising edge of S0 
                    oTG68_RW   <='1';                 -- Pull /RW high to indicate read cycle
                    mystate<=readS1;
--                end if;
                
            when readS1 =>                    
                if amiga_fallingedge = '1' then              -- Falling edge of S1 
                    U3_2OE_C        <=    '0';               -- Enable address bus|                
                    U4_1OE_C        <=    '0';               --                   | 
                    U4_2OE_C        <=    '0';               ----------------------
                    amiga_addr <= cpu_addr(23 downto 1)&'0'; -- Drive a valid address on the address bus
                    mystate<=readS2;
                end if;
                
            when readS2 =>
                if amiga_risingedge = '1' then        -- Rising edge of S2
                    oTG68_ASn    <=    '0';           -- Pull /AS low to indicate that a valid address is on the bus                                                                
                    oTG68_UDSn  <=    '0';            -- Pull /USD,/LDS low|
                    oTG68_LDSn  <=    '0';            ----------------------                                
                    mystate<=readS3;    
                end if;    
                
            when readS3 =>
                if amiga_fallingedge = '1' then       -- Falling edge of S3
                    mystate<=readS4;
                end if;    
                            
            when readS4 =>
                if amiga_risingedge = '1' then        -- Rising edge of S4
                    if DTACK ='0' then                -- Wait for cycle termination signal (DTACK) and inserts wait states
                        mystate<=readS5;
                    end if;
                end if;
                
            when readS5 =>
                if amiga_fallingedge = '1' then       -- Falling edge of S5                                
                    mystate<=readS6;        
                end if;
                
            when readS6 =>
                if amiga_risingedge = '1' then        -- Rising edge of S6
                    U2_1DIR_C <= '0';                 -- Data Bus as input to FPGA|
                    U2_1OE_C  <= '0';                 --                          |
                    U2_2DIR_C <= '0';                 --                          |
                    U2_2OE_C  <= '0';                 -----------------------------
                    cpu_datain<=ioTG68_DATA;          -- Read Data                                        
                    mystate <= readS7; 
                end if;
                    
            when readS7 =>                        
                if amiga_fallingedge = '1' then       -- Rising edge of next S0
                    oTG68_ASn   <='1';                -- Negate /AS,/USD,/LDS|
                    oTG68_UDSn  <='1';                --                     |
                    oTG68_LDSn  <='1';                ------------------------
                    
                    U3_2OE_C     <= '1';              -- Disable address bus|                                    
                    U4_1OE_C     <= '1';              --                    |
                    U4_2OE_C     <= '1';              -----------------------    
                    amiga_addr  <= (others=>'Z');     -- Place Address bus in Hi-Z                            
                    mystate <= delay1;
                end if;        
                            
            when others =>
                null;
        
        end case;
    end if;
end process;

LED <= '1' when sel_fastram = '1' else '0';           -- LED activated on every access to FastRam

END;