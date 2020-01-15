import time
import numpy
import serial
import scipy.signal as signal
import scipy.ndimage.interpolation as interpolation
import scipy.io.wavfile as wavfile

import matplotlib.pyplot as pyplot


#filename           = '..\Data\Messiah.wav'
#filename           = '..\Data\WhenJohnnyComes.wav'
#filename           = '..\Data\TallShips.wav'
filename            = '..\Data\FaithfulInstrumental.wav'

#fpgaPort           = 'com4'
fpgaBaudRate       = 12_000_000

esp32Port          = 'com5'
esp32BaudRate      = 115200 * 8

blockSize          = 3
scale              = 2 ** 21     # 75 kHz frequency deviation
audioLowPassCutOff = 15_000
audioLowPassOrder  = 7
chunkSize          = 20_000
updatePeriod       = 1000

relay              = 0
relayActive        = 0
relayT             = 0

vuLowPassCutoff    = .5
vuLowPassOrder     = 3


def packet(left, right):
    global relay
    global relayActive
    global relayT

    relayDt = .1
   
    pi = 3.14159265359;
    pi2 = pi * 2.;

    rHz = .45 * 5
    gHz = .35 * 5
    bHz = .25 * 5
    
    sinPower = 3.
    vuPower  = 2.

    t = time.time()

    if (t - relayT > relayDt):
        s = numpy.arange(4)
        numpy.random.shuffle(s)

        newActive = int((((left + right) / 2) ** vuPower) * 8)
        
        if newActive > 4: 
            newActive = 4
        
        if (relayActive != newActive):
            relay = 0

            for j in range(newActive):
                relay = relay + (1 << s[j])

            relayT = t
            relayActive = newActive

    data = [relay]
         
    for i in range(12):
        rSin = (numpy.sin(pi2 * (t * rHz + i / 12.)) * 0.5 + 0.5) ** sinPower
        gSin = (numpy.sin(pi2 * (t * gHz + i / 12.)) * 0.5 + 0.5) ** sinPower
        bSin = (numpy.sin(pi2 * (t * bHz + i / 12.)) * 0.5 + 0.5) ** sinPower

        rVu = left  ** vuPower
        gVu = right ** vuPower        
        bVu = ((left + right) / 2) ** vuPower

        r = int((rSin * .75 * rVu + rVu * .25) * 255. + 0.99)
        g = int((gSin * .75 * gVu + gVu * .25) * 255. + 0.99)
        b = int((bSin * .75 * bVu + bVu * .25) * 255. + 0.99)

        if (r > 255):
            print(r)

        if (g > 255):
            print(g)

        if (b > 255):
            print(b)

        data.append(r & 0xFF) 
        data.append(g & 0xFF)
        data.append(b & 0xFF)

    return bytearray(data)


fpga  = serial.Serial(fpgaPort,  baudrate = fpgaBaudRate,  bytesize = 8, parity = 'N', stopbits = 1)
esp32 = serial.Serial(esp32Port, baudrate = esp32BaudRate, bytesize = 8, parity = 'N', stopbits = 1)

print('Loading file \'%s\'...' % filename)

audioRate, wave = wavfile.read(filename)

print('Low-pass filtering...')

b, a = signal.butter(audioLowPassOrder, audioLowPassCutOff / (audioRate / 2.0), 'low')

waveL = signal.filtfilt(b, a, wave[:, 0].astype(numpy.float))
waveR = signal.filtfilt(b, a, wave[:, 1].astype(numpy.float))

print('Resampling...')

symbolRate = fpgaBaudRate / (blockSize * 10)
resampleScale = symbolRate / audioRate
waveL = interpolation.zoom(waveL, zoom = resampleScale, order = 3)
waveR = interpolation.zoom(waveR, zoom = resampleScale, order = 3)

print('Normalizing...')

div = max(numpy.abs(waveL).max(), numpy.abs(waveR).max())

waveL = waveL / div
waveR = waveR / div

print('Generating VU signals...')

b, a = signal.butter(vuLowPassOrder, vuLowPassCutoff / (audioRate / 2.0), 'low')

vuL = signal.filtfilt(b, a, numpy.abs(waveL))
vuL = vuL - vuL.min()
vuL = vuL / vuL.max()

vuR = signal.filtfilt(b, a, numpy.abs(waveR))
vuR = vuR - vuR.min()
vuR = vuR / vuR.max()

print('Multiplexing...')

t = numpy.arange(waveL.shape[0]) / symbolRate
s19 = numpy.sin(2.0 * numpy.pi * 19_000 * t)
s38 = numpy.sin(2.0 * numpy.pi * 38_000 * t)

wave = 0.9 * ((waveL + waveR) / 2.0 + s38 * (waveL - waveR) / 2.0) + 0.1 * s19

print('Signal conditioning...')

wave = wave / numpy.abs(wave).max()
wave = (wave * 0.5) * scale + (2. ** (8 * blockSize - 1))

print('Generating bytestream...')

wave = wave.astype(numpy.int)

w1 =  wave        & 0xFF
w2 = (wave >>  8) & 0xFF
w3 = (wave >> 16) & 0xFF

binWave = list(numpy.reshape(numpy.vstack((w1, w2, w3)).T, (-1)))
stream = bytearray(binWave)

print('Broadcasting...')

data = 0

streamLength = len(stream)
extendedStream = numpy.concatenate((stream, stream[0 : chunkSize - 1]))

while (True):
    position = data % streamLength

    fpga.write(extendedStream[position : position + chunkSize])
    esp32.write(packet(vuL[int(position / blockSize)], vuR[int(position / blockSize)]))

    data = data + chunkSize

    if ((data / chunkSize) % updatePeriod == 0):
        print('[%i s] Uploaded: %.1f Mb' % (10 * data / fpgaBaudRate, data / 1_000_000))

