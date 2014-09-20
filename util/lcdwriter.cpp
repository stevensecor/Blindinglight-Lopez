#include "lcdwriter.h"

LCDWriter::LCDWriter() {
	lcd = DriverStationLCD::GetInstance();
}

LCDWriter::~LCDWriter() {
	lcd->UpdateLCD();
}

void LCDWriter::clear() {
	lcd->Clear();
}

void LCDWriter::line1(const char* format, ...) {
	va_list args;
	va_start(args, format);
	lcd->VPrintfLine(DriverStationLCD::kUser_Line1, format, args);
	va_end(args);
}
void LCDWriter::line2(const char* format, ...) {
	va_list args;
	va_start(args, format);
	lcd->VPrintfLine(DriverStationLCD::kUser_Line2, format, args);
	va_end(args);
}
void LCDWriter::line3(const char* format, ...) {
	va_list args;
	va_start(args, format);
	lcd->VPrintfLine(DriverStationLCD::kUser_Line3, format, args);
	va_end(args);
}
void LCDWriter::line4(const char* format, ...) {
	va_list args;
	va_start(args, format);
	lcd->VPrintfLine(DriverStationLCD::kUser_Line4, format, args);
	va_end(args);
}
void LCDWriter::line5(const char* format, ...) {
	va_list args;
	va_start(args, format);
	lcd->VPrintfLine(DriverStationLCD::kUser_Line5, format, args);
	va_end(args);
}
void LCDWriter::line6(const char* format, ...) {
	va_list args;
	va_start(args, format);
	lcd->VPrintfLine(DriverStationLCD::kUser_Line6, format, args);
	va_end(args);
}
