
setwd("D:/Cortex/KODIZ/kodiz/WPF-Serial-Communication-Advanced-master/Serial Communication WPF/bin/Release")


adcdata <- read_tsv('17.05.2021_17-29.tab', skip = 1 )

adcdata <- read_tsv('19.05.2021_17-45int4Si.tab' )

adcdata <- read_tsv('19.05.2021_17-42int1PhNechaev.tab' )




densityplot(as.numeric(adcdata[adcdata$adc>1000,]$adc))
densityplot(as.numeric(adcdata[adcdata$adc_1>1000,]$adc_1))
