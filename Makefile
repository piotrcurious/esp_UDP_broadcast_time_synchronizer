CC = g++
CFLAGS = -O3 -I. -Imock_arduino
SOURCES = mock_arduino/Arduino.cpp
TARGET_V8 = test_v8
TARGET_V11 = test_v11

all: $(TARGET_V8) $(TARGET_V11)

$(TARGET_V8): mock_arduino/test_v8.cpp $(SOURCES)
	$(CC) $(CFLAGS) mock_arduino/test_v8.cpp -o $(TARGET_V8)

$(TARGET_V11): mock_arduino/test_v11.cpp $(SOURCES)
	$(CC) $(CFLAGS) mock_arduino/test_v11.cpp -o $(TARGET_V11)

clean:
	rm -f $(TARGET_V8) $(TARGET_V11)
