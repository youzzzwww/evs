#pragma once
#include <fstream>

template <typename T>
void write(std::ofstream& stream, const T& t) {
  stream.write((const char*)&t, sizeof(T));
}

template <typename T>
void writeFormat(std::ofstream& stream) {
  write<short>(stream, 1);
}

template <>
void writeFormat<float>(std::ofstream& stream) {
  write<short>(stream, 3);
}

template <typename SampleType>
void writeWAVData(
  char const* outFile,
  SampleType* buf,
  size_t bufSize,
  int sampleRate,
  short channels)
{
  std::ofstream stream(outFile, std::ios::binary);
  stream.write("RIFF", 4);
  write<int>(stream, 36 + bufSize);
  stream.write("WAVE", 4);
  stream.write("fmt ", 4);
  write<int>(stream, 16);
  writeFormat<SampleType>(stream);                                // Format
  write<short>(stream, channels);                                 // Channels
  write<int>(stream, sampleRate);                                 // Sample Rate
  write<int>(stream, sampleRate * channels * sizeof(SampleType)); // Byterate
  write<short>(stream, channels * sizeof(SampleType));            // Frame size
  write<short>(stream, 8 * sizeof(SampleType));                   // Bits per sample
  stream.write("data", 4);
  stream.write((const char*)&bufSize, 4);
  stream.write((const char*)buf, bufSize);
}

void writeWAVData(
  char const* outFile,
  char* buf,
  size_t bufSize,
  int sampleRate,
  int byteRate,
  short channels)
{
  std::ofstream stream(outFile, std::ios::binary);
  stream.write("RIFF", 4);
  write<int>(stream, 36 + bufSize);
  stream.write("WAVE", 4);
  stream.write("fmt ", 4);
  write<int>(stream, 16);
  writeFormat<short>(stream);                                // Format
  write<short>(stream, channels);                                 // Channels
  write<int>(stream, sampleRate);                                 // Sample Rate
  write<int>(stream, sampleRate * channels * byteRate); // Byterate
  write<short>(stream, channels * byteRate);            // Frame size
  write<short>(stream, 8 * byteRate);                   // Bits per sample
  stream.write("data", 4);
  stream.write((const char*)&bufSize, 4);
  stream.write((const char*)buf, bufSize);
}

int addWavHeadToPcmFile(FILE* f_pcm, const char* wav_name, int sampling_rate, int byte_rate,int channels)
{
	char* buffer=NULL;
	unsigned int buffer_size=0;
	
	fseek(f_pcm, 0, SEEK_END);
	buffer_size = ftell(f_pcm);
	fseek(f_pcm, 0, SEEK_SET);

	buffer = (char*)malloc(buffer_size*sizeof(char));
	fread(buffer, buffer_size, 1, f_pcm);
	writeWAVData(wav_name, buffer, buffer_size, sampling_rate, byte_rate, channels);

	return 0;
}