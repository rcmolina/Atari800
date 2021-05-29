rem cas2wav Harrier_Attack.cas /w=s
rem sox Harrier_Attack.wav -r 11111 Harrier_Attack.voc
rem direct Harrier_Attack.voc
rem PAUSE

forfiles /m *.cas /C "cmd /c cas2wav @file /w=s
forfiles /m *.wav /C "cmd /c sox @file -r 11111 @fname_11k.voc
del *.wav
forfiles /m *.voc /C "cmd /c direct @file
del *.voc
mkdir CASTZX
move *.tzx CASTZX
PAUSE
