// https://fhnw.mit-license.org/

let node_id = "testing-envnode-otaa"; // set the node_id here

function bytes2float(hb, lb) {
    return ((hb << 8) + lb) / 100;
}

function byte2mv(batterybyte) {
    return (batterybyte * 2) * 10;
}

function bytes2uint16(hb, lb) {
    return (hb << 8) + lb;
}

var sensors = {
    airth: 0x01,
    soilm: 0x02,
    soilt: 0x03,
    uv: 0x04,
    sound: 0x05,
    turbidity: 0x06,
    distance: 0x07,
    gas: 0x08
}

function Decoder(bytes, port) {
    var decoded = {};
    if (port === 1) {
        decoded.len = bytes.length;
        decoded.vbatt = byte2mv(bytes[0])
        decoded.node_id = node_id;
        var i = 1;
        while (i < bytes.length) {
            switch (bytes[i]) {
                case sensors.airth:
                    decoded.airtemperature = bytes2float(bytes[i + 1], bytes[i + 2]);
                    decoded.airhumidity = bytes2float(bytes[i + 3], bytes[i + 4]);
                    i += 5
                    break;
                case sensors.soilm:
                    decoded.soilmoisture = bytes2uint16(bytes[i + 1], bytes[i + 2]);
                    i += 3
                    break;
                case sensors.soilt:
                    decoded.soiltemperature = bytes2float(bytes[i + 1], bytes[i + 2]);
                    i += 3
                    break;
                case sensors.uv:
                    decoded.uv_index = bytes2float(bytes[i + 1], bytes[i + 2]);
                    i += 3
                    break;
                case sensors.sound:
                    decoded.sound = bytes2uint16(bytes[i + 1], bytes[i + 2]);
                    i += 3
                    break;
                case sensors.turbidity:
                    decoded.turbidity = bytes2uint16(bytes[i + 1], bytes[i + 2]);
                    i += 3
                    break;
                case sensors.distance:
                    decoded.distance = bytes2uint16(bytes[i + 1], bytes[i + 2]);
                    i += 3
                    break;
                case sensors.gas:
                    decoded.gas_NO2 = bytes2uint16(bytes[i + 1], bytes[i + 2]);
                    i += 3;
                    decoded.gas_C2H5OH = bytes2uint16(bytes[i], bytes[i + 1]);
                    i += 2;
                    decoded.gas_VOC = bytes2uint16(bytes[i], bytes[i + 1]);
                    i += 2;
                    decoded.gas_CO = bytes2uint16(bytes[i], bytes[i + 1]);
                    i += 2;
                    break;
                default:
                    i += 1
                    break;
            }
        }
    }
    decoded.measurement = "env";
    return decoded;
}