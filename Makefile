#
# Makefile 
#

PROGRAM = ads1263
VERS = 2.0

# Zdrojové soubory pro program
SRC = ads1263_example.c ads1263_lib.c
OBJ = $(SRC:.c=.o)
HEAD = ads1263_lib.h

# Zdrojové soubory pro knihovnu
LIB_NAME = ads1263
LIB_SRC = ads1263_lib.c
LIB_OBJ = $(LIB_SRC:.c=.o)
STATIC_LIB = lib$(LIB_NAME).a

# C translator (clang, gcc, ..)
CC = clang
AR = ar
RANLIB = ranlib

# Optimalization (-O0 -g = debug, -O0 -pg = gprof, -O2 = normal)
OPT = -O2

# Other parameters (-Wall -Wextra -pedantic)
CFLAGS = -Wall -Wextra $(OPT) #-pedantic 

# Cilum build, install, uninstall, clean a dist neodpovida primo zadny soubor
# (predstirany '.PHONY' target)

.PHONY: build
.PHONY: lib
.PHONY: install
.PHONY: uninstall
.PHONY: clean
.PHONY: dist
.PHONY: test

# list of valid suffixes through the use of the .SUFFIXES special target.
#.SUFFIXES: .c .o

build: $(PROGRAM)

# Sestavení statické knihovny
lib: $(STATIC_LIB)

$(STATIC_LIB): $(LIB_OBJ)
	$(AR) rcs $@ $^
	$(RANLIB) $@

install: build lib
	mkdir -p $(HOME)/lib
	mkdir -p $(HOME)/include
	cp $(STATIC_LIB) $(HOME)/lib
	cp $(HEAD) $(HOME)/include

uninstall:
	rm -f $(HOME)/lib/$(STATIC_LIB)
	rm -f $(HOME)/include/$(HEAD)

clean:
	rm -f *.o $(PROGRAM) $(STATIC_LIB) test_ads1263

dist:
	tar czf $(PROGRAM)-$(VERS).tgz $(SRC) $(HEAD) test_ads1263.c Makefile README.md LICENSE

# Hardware-free test (emulated ADS1263)
test: test_ads1263.c $(LIB_SRC) $(HEAD)
	$(CC) $(CFLAGS) test_ads1263.c $(LIB_SRC) -lm -o test_ads1263
	./test_ads1263

$(PROGRAM): $(OBJ) Makefile
	$(CC) $(OBJ) -o $(PROGRAM)

%.o: %.c $(HEAD) Makefile
	$(CC) $(CFLAGS) -c $<
