import os
from .base_parser import FlightData
from .ulog_parser import UlogParser
from .blackbox_parser import BlackboxParser
from .ardulog_parser import ArduLogParser

PARSERS = [UlogParser(), BlackboxParser(), ArduLogParser()]


def detect_and_parse(filepath: str) -> FlightData:
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"파일을 찾을 수 없습니다: {filepath}")

    for parser in PARSERS:
        if parser.can_parse(filepath):
            return parser.parse(filepath)

    ext = os.path.splitext(filepath)[1].lower()
    raise ValueError(
        f"지원하지 않는 파일 형식입니다: {ext}\n"
        f"지원 형식: .ulg, .ulog (PX4), .bbl, .bfl, .csv (Betaflight/iNav), .bin, .log (ArduPilot)"
    )
