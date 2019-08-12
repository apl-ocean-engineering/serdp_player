# serdp_player

Serdp_player .mov decoder to read .movs and decode them into GPMF and image formats, and displays. Main tool only reads and displays, core library contains .mov parsing code. 

Fips makes it easy:
$./fips gen
$./fips build
$./fips run serdp_player -- <path_to_mov>
