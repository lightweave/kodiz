library(serial)

rm(list = ls())         # clear environment
graphics.off()          # close all graphic windows



### establish a serial connection

con <- serialConnection(name = "get_temps",
                        port = "COM6",
                        mode = "115200,n,8,1",
                        buffering = "none",
                        newline = 1,
                        translation = "cr")



#close(con)
isOpen(con)
open(con)
isOpen(con)



### grab data from serial port
read.serialConnection(con)

#Grab last five characters and turn into number
substrRight <- function(x, n){ substr(x, nchar(x)-n+1, nchar(x)) }

tmp <- read.serialConnection(con)
tmp
substrRight(tmp,5)
as.numeric(substrRight(tmp,5))

as.numeric(substrRight(read.serialConnection(con),5))




####  Single Non-Moving Plot

x11()

N <- 1000
x <- rep(NA,N)

for(i in seq(N)) {
  Sys.sleep(0.101)
  
  tmp <- read.serialConnection(con)
  x[i] <- as.numeric(substrRight(tmp,5))
  plot(1:N, x, type="l", ylim=c(22, 28),lwd=2,
       main="Temperature from Arduino", xlab="Seconds*10", 
       ylab="Temperature - Celsius")
}




#### Scrolling Plot

x11()
N <- 1000
x <- rep(NA,N)

for(i in seq(N)) {
  Sys.sleep(0.101)
  
  tmp <- read.serialConnection(con)
  x[i] <- as.numeric(substrRight(tmp,5))
  
  if(i<=200){
    plot(1:200, x[1:200], type="l", ylim=c(22, 28),lwd=2,
         main="Temperature from Arduino", xlab="Seconds*10", 
         ylab="Temperature - Celsius")
  } else
    
    if(i>200){
      plot((i-200):i, x[(i-200):i], type="l", ylim=c(22, 28),lwd=2,
           main="Temperature from Arduino", xlab="Seconds*10", 
           ylab="Temperature - Celsius")
    } 
  
  
}
