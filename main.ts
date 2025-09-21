////////////////////
//################//
//##            ##//
//##  track.ts  ##//
//##            ##//
//################//
////////////////////

enum TrackType {
    //% block="black on white"
    //% block.loc.nl="zwart op wit"
    BlackOnWhite,
    //% block="white on black"
    //% block.loc.nl="wit op zwart"
    WhiteOnBlack,
}

enum TrackMask {
    //% block="two sensors"
    //% block.loc.nl="twee sensoren"
    Track2 = 10,
    //% block="three sensors"
    //% block.loc.nl="drie sensoren"
    Track3 = 14,
    //% block="four sensors"
    //% block.loc.nl="vier sensoren"
    Track4 = 27,
    //% block="five sensors"
    //% block.loc.nl="vijf sensoren"
    Track5 = 31,
}

enum Track {
    //% block="off the track"
    //% block.loc.nl="van de lijn af"
    OffTrack = 0,
    //% block="the track at far left"
    //% block.loc.nl="de lijn op uiterst links"
    FarLeft = 1,
    //% block="the track at left"
    //% block.loc.nl="de lijn op links"
    Left = 2,
    //% block="on the track"
    //% block.loc.nl="op de lijn"
    Mid = 4,
    //% block="the track at right"
    //% block.loc.nl="de lijn op rechts"
    Right = 8,
    //% block="the track at far right"
    //% block.loc.nl="de lijn op uiterst rechts"
    FarRight = 16,
}

function trackPosition(track: number, mask = TrackMask.Track2, tracktype = TrackType.BlackOnWhite): Track {
    if (tracktype == TrackType.WhiteOnBlack) track = ~track
    track = (track & mask)

    if (!track)
        return Track.OffTrack
    if (track & 17) { // extreme left or right sensor
        if (track & 4) { // mid sensor too
            if (track & 1) return Track.Left
            if (track & 16) return Track.Right
        }
        else { // whitout mid sensor
            if (track & 1) return Track.FarLeft
            if (track & 16) return Track.FarRight
        }
    }
    if (((track & 10) == 10) ||   // both left and right sensor
        ((track & 4) == track)) // mid sensor only
        return Track.Mid
    if (track & 2)
        return Track.Left
    if (track & 8)
        return Track.Right
    return Track.OffTrack
}


////////////////////
//################//
//##            ##//
//##  servo.ts  ##//
//##            ##//
//################//
////////////////////

enum ServoType {
    Continuous = 0,
    ST90 = 90,
    ST180 = 180,
    ST270 = 270,
    ST360 = 360,
}

namespace Servo {

    export class Device {

        pin: AnalogPin
        servo: ServoType
        minpw: number = 1000
        maxpw: number = 2000

        constructor(_pin: AnalogPin, _type: ServoType) {
            this.pin = _pin
            this.servo = _type
        }

        setPulse(_min: number, _max: number) {
            this.minpw = _min;
            this.maxpw = _max;
        }

        angle(_angle: number) {
            _angle = Math.map(_angle, this.minpw, this.maxpw, 0, this.servo)
            pins.servoSetPulse(this.pin, _angle)
            //pins.servoWritePin(this.pin, _angle)
        }

        speed(_speed: number) {
            _speed = Math.map(_speed, this.minpw, this.maxpw, -100, 100)
            pins.servoSetPulse(this.pin, _speed)
        }
    }

    export function create(_pin: AnalogPin, _type: ServoType): Device {
        let device = new Device(_pin, _type)
        return device
    }
}


/////////////////////////
//#####################//
//##                 ##//
//##  cutebotpro.ts  ##//
//##                 ##//
//#####################//
/////////////////////////

enum Led {
    //% block="left led"
    //% block.loc.nl="linker led"
    Left,
    //% block="right led"
    //% block.loc.nl="rechter led"
    Right,
    //% block="both leds"
    //% block.loc.nl="beide leds"
    Both
}

enum ServoPort {
    S1,
    S2,
    S3,
    S4,
}

enum GpioPort {
    G1,
    G2,
    G3,
    G4,
}

namespace CutebotPro {
    // supports CutebotPro V2

    const cutebotProAddr = 0x10

    let trackType = TrackType.BlackOnWhite

    let AnalogGP = [AnalogPin.P1, AnalogPin.P2, AnalogPin.P13, AnalogPin.P14]
    let DigitalGP = [DigitalPin.P1, DigitalPin.P2, DigitalPin.P13, DigitalPin.P14]

    function delay_ms(ms: number) {
        let endTime = input.runningTime() + ms;
        while (endTime > input.runningTime()) { }
    }

    export function pid_delay_ms(ms: number) {
        let time = control.millis() + ms
        while (1) {
            i2cCommandSend(0xA0, [0x05])
            if (pins.i2cReadNumber(cutebotProAddr, NumberFormat.UInt8LE, false) || control.millis() >= time) {
                basic.pause(500)
                break
            }
            basic.pause(10)
        }
    }

    export function i2cCommandSend(command: number, params: number[]) {
        let buff = pins.createBuffer(params.length + 4);
        buff[0] = 0xFF;
        buff[1] = 0xF9;
        buff[2] = command;
        buff[3] = params.length;
        for (let i = 0; i < params.length; i++) {
            buff[i + 4] = params[i];
        }
        pins.i2cWriteBuffer(cutebotProAddr, buff);
        delay_ms(1);
    }

    // MOTION MODULE

    export function speed(left: number, right: number): void {
        // speed in % [-100, 100]

        let direction: number = 0;
        if (left < 0) direction |= 0x01;
        if (right < 0) direction |= 0x02;
        i2cCommandSend(0x10, [2, Math.abs(left), Math.abs(right), direction]);
    }

    export function move(speed: number, distance: number): void {
        // speed in % [-100, -40] backward and [40, 100] forward
        // distance in cm [0, 6000]

        distance = ((distance > 6000 ? 6000 : distance) < 0 ? 0 : distance);
        distance *= 10 // cm to mm
        let distance_h = distance >> 8;
        let distance_l = distance & 0xFF;

        let direction2: number
        if (speed <= 0) {
            speed = -speed
            direction2 = 3
        } else
            direction2 = 0

        speed *= 5 // % to mm/s
        speed = ((speed > 500 ? 500 : speed) < 200 ? 200 : speed);
        let speed_h = speed >> 8;
        let speed_l = speed & 0xFF;

        i2cCommandSend(0x84, [distance_h, distance_l, speed_h, speed_l, direction2]);
        pid_delay_ms(Math.round(distance * 1.0 / 1000 * 8000 + 3000))
    }

    // MOTOR MODULE

    export function motor(speed: number): void {
        let direction: number = (speed > 0 ? 1 : 0)
        i2cCommandSend(0x30, [Math.abs(speed), direction])
    }

    // SERVO MODULE

    export function servoAngle(port: ServoPort, angle: number, _type: ServoType = ServoType.ST180) {
        angle = Math.map(angle, 0, _type, 0, 180)
        i2cCommandSend(0x40, [port, angle])
    }

    export function servoSpeed(port: ServoPort, speed: number) {
        speed = Math.map(speed, -100, 100, 0, 180)
        i2cCommandSend(0x40, [port, speed])
    }

    // LED MODULE

    export function ledColor(led: Led, color: Color): void {
        let rgbval = fromColor(color)
        let red = (rgbval >> 16) & 0xFF;
        let green = (rgbval >> 8) & 0xFF;
        let blue = (rgbval) & 0xFF;
        i2cCommandSend(0x20, [led, red, green, blue]);
    }

    // TRACKING MODULE

    export function setTrackType(_type: TrackType) {
        trackType = _type
    }

    export function readTrack(): Track {
        i2cCommandSend(0x60, [0x00])
        let state = pins.i2cReadNumber(cutebotProAddr, NumberFormat.UInt8LE, true)
        // From left to right the track sensors represent a bit in 'state'.
        // This agrees with the 'Track' extension. So use it without conversion.
        let track = (state & 3) + ((state & 12) << 1)
        track = trackPosition(track, TrackMask.Track4, trackType)
        return track
    }

    export function isTrackAtLeft(): boolean {
        let state = readTrack()
        let track = trackPosition(state, TrackMask.Track4, trackType)
        return (track == Track.Left || track == Track.FarLeft)
    }

    export function isTrackAtRight(): boolean {
        let state = readTrack()
        let track = trackPosition(state, TrackMask.Track4, trackType)
        return (track == Track.Right || track == Track.FarRight)
    }

    export function isOnTrack(): boolean {
        let state = readTrack()
        let track = trackPosition(state, TrackMask.Track4, trackType)
        return (track == Track.Mid)
    }

    export function isOffTrack(): boolean {
        let state = readTrack()
        let track = trackPosition(state, TrackMask.Track4, trackType)
        return (track == Track.OffTrack)
    }

    // DISTANCE MODULE

    export function readDistance(): number {
        // send pulse

        pins.setPull(DigitalPin.P8, PinPullMode.PullNone);
        pins.digitalWritePin(DigitalPin.P8, 0);
        control.waitMicros(2);
        pins.digitalWritePin(DigitalPin.P8, 1);
        control.waitMicros(10);
        pins.digitalWritePin(DigitalPin.P8, 0);

        // read pulse

        // the next code is replacing the original since
        // driving the motors causes interference with pulseIn

        while (!pins.digitalReadPin(DigitalPin.P12)) { }
        let tm1 = input.runningTimeMicros()
        while (pins.digitalReadPin(DigitalPin.P12)) {
            if (input.runningTimeMicros() - tm1 > 7288)
                return 999 // timeout at further than 250 cm
        }
        let tm2 = input.runningTimeMicros()
        let dist = (tm2 - tm1) * 343 / 20000
        return Math.floor(dist)
    }

    // GPIO MODULE

    export function analogPin(port: GpioPort): AnalogPin {
        return AnalogGP[port]
    }

    export function digitalPin(port: GpioPort): DigitalPin {
        return DigitalGP[port]
    }
}


//////////////////////////
//######################//
//##                  ##//
//##  sumo-player.ts  ##//
//##                  ##//
//######################//
//////////////////////////

let ROLE = Player.Green

enum Bend {
    //% block="go straight"
    //% block.loc.nl="ga rechtdoor"
    None,
    //% block="turn to the left"
    //% block.loc.nl="draai naar links"
    Left,
    //% block="turn to the right"
    //% block.loc.nl="draai naar rechts"
    Right
}

enum Side {
    //% block="one of the sides"
    //% block.loc.nl="één van beide kanten"
    Both,
    //% block="the left side"
    //% block.loc.nl="de linker kant"
    Left,
    //% block="the right side"
    //% block.loc.nl="de rechter kant"
    Right
}

// setPlayerHandler is called from match.ts
setPlayerHandler = (player: Player) => {
    ROLE = player
}

// showPlayerHandler is called from match.ts and this extension
showPlayerHandler = () => {
    if (ROLE == Player.Green)
        CutebotPro.ledColor(Led.Both, Color.Green)
    else
        CutebotPro.ledColor(Led.Both, Color.Blue)
}

resetHandler = () => {
    SumoPlayer.stop()
}

stopHandler = () => {
    SumoPlayer.stop()
}

if (initPlayerHandler) initPlayerHandler()
if (displayHandler) displayHandler()
CutebotPro.setTrackType(TrackType.WhiteOnBlack)

let outOfFieldHandler: handler

basic.forever(function () {
    if (isPlaying && !isPlaying()) return
    if (CutebotPro.readTrack() != Track.OffTrack) {
        CutebotPro.ledColor(Led.Both, Color.Red)
        if (outOfFieldHandler) outOfFieldHandler()
    }
    else {
        if (showPlayerHandler) showPlayerHandler()
    }
})

//% color="#00CC00" icon="\uf1f9"
//% block="Sumo"
//% block.loc.nl="Sumo"
namespace SumoPlayer {

    let fielddiameter = 40 // cm

    //% color="#FFCC00"
    //% block="when the robot is out of the field"
    //% block.loc.nl="wanneer de robot buiten het speelveld is"
    export function onOutOfField(code: () => handler) {
        outOfFieldHandler = code()
    }

    //% block="at %side out of the field"
    //% block="aan %side uit het veld"
    export function isOutOfField(side: Side): boolean {
        switch (side) {
            case Side.Both:     return !CutebotPro.isOffTrack()
            case Side.Left:     return CutebotPro.isTrackAtLeft()
            case Side.Right:    return CutebotPro.isTrackAtRight()
        }
        return false
    }

    //% block="stop"
    //% block.loc.nl="stop"
    export function stop() {
        CutebotPro.speed(0, 0)
    }

    //% block="move %dir and %bend"
    //% block.loc.nl="rijd %dir en %bend"
    export function move(dir: Move, bend: Bend) {
        let speed: number
        if (dir == Move.Forward) speed = 20
        else speed = -20
        switch (bend) {
            case Bend.None: CutebotPro.speed(speed, speed); break;
            case Bend.Left: CutebotPro.speed(speed / 2, speed); break;
            case Bend.Right: CutebotPro.speed(speed, speed / 2); break;
        }
    }

    //% block="push the opponent"
    //% block.loc.nl="duw de tegenstander"
    export function pushOpponent() {
        if (CutebotPro.readDistance() > 10) return
        CutebotPro.speed(100, 100)
    }

    //% block="run to the opponent"
    //% block.loc.nl="rijd naar de tegenstander"
    export function runToOpponent() {
        let cm: number
        CutebotPro.speed(20, 20)
        do {
            if (isPlaying && !isPlaying()) return
            cm = CutebotPro.readDistance()
            basic.pause(10)
        } while (cm > 10 && cm < fielddiameter)
        CutebotPro.speed(0, 0)
    }

    //% block="turn to the opponent"
    //% block.loc.nl="draai richting tegenstander"
    export function findOpponent() {
        CutebotPro.speed(-15, 15)
        while (CutebotPro.readDistance() > fielddiameter) {
            if (isPlaying && !isPlaying()) return
            basic.pause(1)
        }
        CutebotPro.speed(0, 0)
    }

    //% subcategory="Show"
    //% color="#00CC00"
    //% block="turn %led color %color"
    //% block.loc.nl="kleur %led %color"
    //% color.defl=Color.White
    export function showColor(led: Led, color: Color) {
        CutebotPro.ledColor(led, color)
    }

    //% subcategory="Show"
    //% color="#00CC00"
    //% block="turn both leds off"
    //% block.loc.nl="schakel beide leds uit"
    export function ledsOff() {
        CutebotPro.ledColor(Led.Both, Color.None)
    }

    //% subcategory="Show"
    //% color="#FFCC44"
    //% block="tornado"
    //% block.loc.nl="tornado"
    export function tornado() {
        let on = true
        for (let speed = 10; speed < 75; speed += 5) {
            if (on)
                CutebotPro.ledColor(Led.Both, Color.Magenta)
            else
                if (showPlayerHandler) showPlayerHandler()
            on = !on
            CutebotPro.speed(speed, -speed)
            basic.pause(168)
        }
        for (let speed = 75; speed >= 0; speed -= 5) {
            if (on)
                CutebotPro.ledColor(Led.Both, Color.Magenta)
            else
                if (showPlayerHandler) showPlayerHandler()
            on = !on
            CutebotPro.speed(speed, -speed)
            basic.pause(168)
        }
        if (showPlayerHandler) showPlayerHandler()
    }

    //% subcategory="Show"
    //% color="#FFCC44"
    //% block="shake"
    //% block.loc.nl="schudden"
    export function shake() {
        for (let i = 0; i < 6; i++) {
            CutebotPro.ledColor(Led.Both, Color.Magenta)
            CutebotPro.speed(30, 30)
            basic.pause(200)
            if (showPlayerHandler) showPlayerHandler()
            CutebotPro.speed(-30, -30)
            basic.pause(230)
        }
        CutebotPro.speed(0, 0)
        if (showPlayerHandler) showPlayerHandler()
    }
}
