
setwd("D:/Cortex/KODIZ/kodiz/WPF-Serial-Communication-Advanced-master/Serial Communication WPF/bin/Release")


adcdata <- read_tsv('17.05.2021_17-29.tab', skip = 1 )

adcdata <- read_tsv('20.05.2021_12-51_int_1_VB.tab' )

adcdata <- read_tsv('19.05.2021_17-42int1PhNechaev.tab' )
# значения ацп прочитались символами, преобразуем в числа
adcdata$adc <- as.numeric(adcdata$adc)
adcdata$adc_1 <- as.numeric(adcdata$adc_1)

# обираем совпадения
coinsedence<-filter(adcdata,coincidence > 0, adc>0) 
# диаграмма рассеяния для совпадений
plot(4000 - coinsedence$adc, 4000 - coinsedence$adc_1, 
   #  xlim = c(0,4200),ylim = c(0,4200),
     main = "Диаграмма рассеяния для совпадений")


histogram(4000 - coinsedence$adc, nint = length(4000 - coinsedence$adc), type = "count",
          main = "Спектр первого детектора при совпадениях")

histogram(4000 - coinsedence$adc_1, nint = length(4000 - coinsedence$adc_1), type = "count",
          main = "Спектр второго детектора при совпадениях")

#  первый детектор
test1<-filter(adcdata,detnuber==0, adc>1000) 
densityplot(as.numeric(test1$adc))
histogram(as.numeric(test1$adc), nint = length(test1$adc))

# второй детектор
test2<-filter(adcdata,detnuber==08, adc>1000) 
densityplot(as.numeric(test2$adc))

densityplot(as.numeric(adcdata[adcdata$adc_1>1000,]$adc_1))



# отбираем массив без совпадений
noncoins<-filter(adcdata,coincidence == 0)

# отбираем первый детектор
det1 <-   filter(noncoins, detnuber==0) 
# таблица состоит из двух половин левой и правой разделяем из пополам
det1 <-   det1[,c(1,4, 5,6 ) ]
work <-   filter(noncoins, detnuber_1==0) 
work <-   work[,c(1,9, 10, 11 ) ]
# делаем одинаковые заголовки
names(work)<- names(det1)
# прикладываем правую таблицу к левой снизу
det1 <- rbind(det1, work)
# фильтруем значения 
det1 <-   filter(det1, adc>0) 
# гистограмма значений ацп
histogram(4000 - det1$adc, nint = length(4000 - det1$adc), type = "count",
          main = "Спектр первого детектора без совпадений")


# отбираем второй детектор
det2 <-   filter(noncoins, detnuber > 0) 
det2 <-   det2[,c(1,4, 5,6 ) ]
work <-   filter(noncoins, detnuber_1>0) 
work <-   work[,c(1,9, 10, 11 ) ]
names(work)<- names(det2)
det2 <- rbind(det2, work)
det2 <-   filter(det2, adc>0) 
histogram(4000 - det2$adc, nint = length(4000 - det2$adc), type = "count",
          main = "Спектр второго детектора без совпадений")
