function Decoder(bytes, port) {
  var decoded = {};
  switch (port) {
    case 1:
      decoded.lat  = (bytes[0] + (bytes[1] << 8) + (bytes[2] << 16)) / 10000;
      decoded.lon  = (bytes[3] + (bytes[4] << 8) + (bytes[5] << 16)) / 10000;
      decoded.alt  = bytes[6];
      decoded.hdop = bytes[7]/100;
      break;
  }
  return decoded;
}
