CC = g++
CFLAGS=-fPIC -Wall
LDFLAGS=-lm
EXECUTABLE = test

# Fichiers sources du projet
SOURCES = main.cpp Robot.cpp Motor.cpp MPU6050.cpp Ultrasonic.cpp Encoder.cpp

# Transforme les noms des fichiers sources en noms d'objets
OBJECTS = $(SOURCES:.cpp=.o)

# Règle par défaut : construit l'exécutable
all: $(EXECUTABLE)

# Règle pour la construction de l'exécutable
$(EXECUTABLE): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@ -lwiringPi -lpigpio -li2c -lpthread
	gpio -g mode 18 alt5
	gpio -g mode 19 alt5

# Règle générique pour la construction des fichiers objets
%.o: %.cpp
	$(CC) $(CFLAGS) -c $< -o $@

# Dépendances
main.o: Robot.h
Robot.o: Robot.h Motor.h MPU6050.h Ultrasonic.h Encoder.h
Motor.o: Motor.h
MPU6050.o: MPU6050.h
Ultrasonic.o: Ultrasonic.h
Encoder.o: Encoder.h

# Nettoie les fichiers objets et l'exécutable
clean:
	rm -f $(OBJECTS) $(EXECUTABLE)
