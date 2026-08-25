# Host-сборка Refloat с mock-платформой VESC.
#
# Доказывает главный тезис Strategy B: исходники Refloat собираются на другой
# архитектуре без единого изменения, достаточно подменить vesc_c_if.h и
# реализовать 75 функций.
#
# Использование:
#   make            собрать
#   make test       собрать и прогнать все сценарии
#   make gen        перегенерировать conf/* из settings.xml

ROOT     := $(abspath $(CURDIR)/../..)
UPSTREAM := $(ROOT)/refloat-upstream
RSRC     := $(UPSTREAM)/src
GEN      := $(ROOT)/build/gen
OBJ      := $(ROOT)/build/host
BIN      := $(OBJ)/refloat_host_tests

VERSION      := $(shell cat $(UPSTREAM)/version)
PACKAGE_NAME := $(shell cat $(UPSTREAM)/package_name)
GIT_HASH     := $(shell git -C $(UPSTREAM) rev-parse --short=8 HEAD)

# led_driver.c исключён: это единственный непортируемый файл Refloat (регистры STM32).
# Его заменяет mock/led_driver_host.c с тем же интерфейсом.
REFLOAT_SRC := $(filter-out $(RSRC)/led_driver.c, $(wildcard $(RSRC)/*.c)) \
               $(wildcard $(RSRC)/filters/*.c) \
               $(wildcard $(RSRC)/lib/*.c) \
               $(RSRC)/conf/buffer.c

GEN_SRC  := $(GEN)/conf/confparser.c $(GEN)/conf/confxml.c

HOST_SRC := mock/mock_vesc_if.c \
            mock/logical_motor_mock.c \
            mock/led_driver_host.c \
            harness/refloat_facade.c \
            scenarios.c \
            runner.c

SOURCES := $(REFLOAT_SRC) $(GEN_SRC) $(HOST_SRC)
OBJECTS := $(patsubst %,$(OBJ)/%.o,$(notdir $(basename $(SOURCES))))

CC      ?= cc
CSTD    := -std=gnu99
WARN    := -Wall -Wextra -Wundef -Wno-unused-parameter
# -fsingle-precision-constant обязателен и на ESP32: иначе double-литералы в
# контуре баланса уходят в программную эмуляцию (docs/threading_model.md, R-17).
FLOATFL := $(shell $(CC) -Werror -fsingle-precision-constant -x c -c /dev/null -o /dev/null 2>/dev/null \
             && echo -fsingle-precision-constant)
INCLUDE := -I$(CURDIR)/mock -I$(CURDIR)/harness -I$(RSRC) -I$(RSRC)/conf -I$(GEN)
CFLAGS  := $(CSTD) $(WARN) $(FLOATFL) -g -O1 -include stddef.h $(INCLUDE)
LDFLAGS := -lm -lpthread

VPATH := $(RSRC):$(RSRC)/filters:$(RSRC)/lib:$(RSRC)/conf:$(GEN)/conf:$(CURDIR):$(CURDIR)/mock:$(CURDIR)/harness

.PHONY: all test gen clean

all: $(BIN)

gen: $(GEN)/conf/confparser.c

$(GEN)/conf/confparser.c: $(RSRC)/conf/settings.xml $(ROOT)/tools/gen_conf.py
	@mkdir -p $(GEN)
	python3 $(ROOT)/tools/gen_conf.py \
		--settings $(RSRC)/conf/settings.xml \
		--out $(GEN) \
		--version $(VERSION) \
		--package-name "$(PACKAGE_NAME)" \
		--git-hash $(GIT_HASH)

$(GEN)/conf/confxml.c: $(GEN)/conf/confparser.c

$(OBJ)/%.o: %.c | $(GEN)/conf/confparser.c
	@mkdir -p $(OBJ)
	$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(BIN): $(OBJECTS)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

test: $(BIN)
	$(BIN)

clean:
	rm -rf $(OBJ)

-include $(OBJ)/*.d
