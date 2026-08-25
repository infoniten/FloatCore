# FloatCore — сборка host-инструментов.
#
#   make                 собрать всё
#   make test            прогнать протокольные и host-тесты
#   make host            собрать и запустить FloatCore Host (мост к VESC Tool)
#   make gen             перегенерировать conf/* и qml_app.* из upstream
#   make clean
#
# Прошивка для ESP32 здесь не собирается: платы ещё нет, и на этом этапе
# вывод на моторы запрещён (ТЗ v0.4 §10).

ROOT     := $(abspath $(CURDIR))
UPSTREAM := $(ROOT)/refloat-upstream
RSRC     := $(UPSTREAM)/src
GEN      := $(ROOT)/build/gen
OBJ      := $(ROOT)/build/obj
BIN      := $(ROOT)/build

VERSION      := $(shell cat $(UPSTREAM)/version 2>/dev/null)
PACKAGE_NAME := $(shell cat $(UPSTREAM)/package_name 2>/dev/null)
GIT_HASH     := $(shell git -C $(UPSTREAM) rev-parse --short=8 HEAD 2>/dev/null)

CC      ?= cc
CSTD    := -std=gnu99
WARN    := -Wall -Wextra -Wundef -Wno-unused-parameter
# clang не поддерживает -fsingle-precision-constant, поэтому флаг определяется пробой.
FLOATFL := $(shell $(CC) -Werror -fsingle-precision-constant -x c -c /dev/null -o /dev/null \
             2>/dev/null && echo -fsingle-precision-constant)

# -O1 — минимум: src/time.h объявляет функции как C99 inline без внешнего
# определения, на -O0 они не находятся линковщиком.
# -include stddef.h — src/conf/buffer.c рассчитывает на транзитивный size_t.
BASE_CFLAGS := $(CSTD) $(WARN) $(FLOATFL) -g -O1

# --- Refloat (upstream, не изменяется) --------------------------------------
# led_driver.c исключён: единственный непортируемый файл (регистры STM32).
REFLOAT_SRC := $(filter-out $(RSRC)/led_driver.c, $(wildcard $(RSRC)/*.c)) \
               $(wildcard $(RSRC)/filters/*.c) \
               $(wildcard $(RSRC)/lib/*.c) \
               $(RSRC)/conf/buffer.c
GEN_SRC     := $(GEN)/conf/confparser.c $(GEN)/conf/confxml.c

# --- Протокольный слой (без платформенных зависимостей) ----------------------
PROTO_SRC := $(wildcard $(ROOT)/compat/vesc_protocol/*.c) \
             $(wildcard $(ROOT)/compat/vesc_protocol/generated/*.c) \
             $(ROOT)/compat/config/floatcore_limits.c

# --- Mock-платформа ----------------------------------------------------------
MOCK_SRC := $(ROOT)/compat/config/floatcore_limits.c \
            $(ROOT)/tests/host/mock/mock_vesc_if.c \
            $(ROOT)/tests/host/mock/logical_motor_mock.c \
            $(ROOT)/tests/host/mock/led_driver_host.c \
            $(ROOT)/tests/host/harness/refloat_facade.c

REFLOAT_INC := -I$(ROOT)/tests/host/mock -I$(ROOT)/tests/host/harness \
               -I$(RSRC) -I$(RSRC)/conf -I$(GEN)

HOST_TESTS_BIN := $(BIN)/refloat_host_tests
PROTO_TESTS_BIN := $(BIN)/protocol_tests
HOST_BIN := $(BIN)/floatcore_host

.PHONY: all test test-all integration gen clean host host-tests protocol-tests

all: $(HOST_TESTS_BIN) $(PROTO_TESTS_BIN) $(HOST_BIN)

# ----------------------------------------------------------------- генерация

gen: $(GEN)/conf/confparser.c $(GEN)/qml_app.c

$(GEN)/conf/confparser.c: $(RSRC)/conf/settings.xml $(ROOT)/tools/gen_conf.py
	@mkdir -p $(GEN)
	python3 $(ROOT)/tools/gen_conf.py --settings $(RSRC)/conf/settings.xml --out $(GEN) \
		--version $(VERSION) --package-name "$(PACKAGE_NAME)" --git-hash $(GIT_HASH)

$(GEN)/conf/confxml.c: $(GEN)/conf/confparser.c

$(GEN)/qml_app.c: $(UPSTREAM)/ui.qml.in $(ROOT)/tools/gen_qml.py
	@mkdir -p $(GEN)
	python3 $(ROOT)/tools/gen_qml.py --qml-in $(UPSTREAM)/ui.qml.in --out $(GEN) \
		--package-name "$(PACKAGE_NAME)" --version $(VERSION)

$(GEN)/qml_app.h: $(GEN)/qml_app.c

# ------------------------------------------------------- protocol_tests (чистый)
# Линкуется ТОЛЬКО с compat/vesc_protocol — это и есть доказательство того,
# что протокольный слой не зависит ни от платформы, ни от Refloat.

PROTO_OBJ := $(patsubst %,$(OBJ)/proto_%.o,$(notdir $(basename $(PROTO_SRC)))) \
             $(OBJ)/proto_test_protocol.o

$(OBJ)/proto_%.o: $(ROOT)/compat/vesc_protocol/%.c
	@mkdir -p $(OBJ)
	$(CC) $(BASE_CFLAGS) -MMD -MP -c $< -o $@

$(OBJ)/proto_%.o: $(ROOT)/compat/vesc_protocol/generated/%.c
	@mkdir -p $(OBJ)
	$(CC) $(BASE_CFLAGS) -MMD -MP -c $< -o $@

$(OBJ)/proto_%.o: $(ROOT)/compat/config/%.c
	@mkdir -p $(OBJ)
	$(CC) $(BASE_CFLAGS) -MMD -MP -c $< -o $@

$(OBJ)/proto_test_protocol.o: $(ROOT)/tests/protocol/test_protocol.c
	@mkdir -p $(OBJ)
	$(CC) $(BASE_CFLAGS) -MMD -MP -c $< -o $@

$(PROTO_TESTS_BIN): $(PROTO_OBJ)
	@mkdir -p $(BIN)
	$(CC) $^ -lm -o $@

protocol-tests: $(PROTO_TESTS_BIN)

# ------------------------------------------------------------ host-тесты Refloat

HT_SRC := $(REFLOAT_SRC) $(GEN_SRC) $(MOCK_SRC) \
          $(ROOT)/tests/host/scenarios.c $(ROOT)/tests/host/runner.c
HT_OBJ := $(patsubst %,$(OBJ)/ht_%.o,$(notdir $(basename $(HT_SRC))))

HT_VPATH := $(RSRC) $(RSRC)/filters $(RSRC)/lib $(RSRC)/conf $(GEN)/conf $(ROOT)/tests/host $(ROOT)/tests/host/mock $(ROOT)/tests/host/harness

$(OBJ)/ht_%.o: $(ROOT)/compat/config/%.c
	@mkdir -p $(OBJ)
	$(CC) $(BASE_CFLAGS) -MMD -MP -c $< -o $@

$(OBJ)/ht_%.o: %.c | $(GEN)/conf/confparser.c
	@mkdir -p $(OBJ)
	$(CC) $(BASE_CFLAGS) -include stddef.h $(REFLOAT_INC) -MMD -MP -c $< -o $@

vpath %.c $(HT_VPATH)

$(HOST_TESTS_BIN): $(HT_OBJ)
	@mkdir -p $(BIN)
	$(CC) $^ -lm -lpthread -o $@

host-tests: $(HOST_TESTS_BIN)

# --------------------------------------------------------------- FloatCore Host
# Важно: файлы platform/host и compat собираются БЕЗ -I$(RSRC).
# В src/ у Refloat лежит собственный time.h, который иначе перекрыл бы
# системный <time.h> (та самая изоляция, описанная в docs/threading_model.md).

FH_SRC := $(REFLOAT_SRC) $(GEN_SRC) $(filter-out $(ROOT)/compat/config/floatcore_limits.c,$(MOCK_SRC)) $(PROTO_SRC) \
          $(GEN)/qml_app.c \
          $(ROOT)/platform/host/tcp_transport.c $(ROOT)/platform/host/floatcore_host.c
FH_OBJ := $(patsubst %,$(OBJ)/fh_%.o,$(notdir $(basename $(FH_SRC))))

PLAIN_CFLAGS := $(BASE_CFLAGS) -I$(GEN)

$(OBJ)/fh_%.o: $(ROOT)/platform/host/%.c | $(GEN)/qml_app.h
	@mkdir -p $(OBJ)
	$(CC) $(PLAIN_CFLAGS) -MMD -MP -c $< -o $@

$(OBJ)/fh_%.o: $(ROOT)/compat/vesc_protocol/%.c
	@mkdir -p $(OBJ)
	$(CC) $(PLAIN_CFLAGS) -MMD -MP -c $< -o $@

$(OBJ)/fh_%.o: $(ROOT)/compat/vesc_protocol/generated/%.c
	@mkdir -p $(OBJ)
	$(CC) $(PLAIN_CFLAGS) -MMD -MP -c $< -o $@

$(OBJ)/fh_%.o: $(ROOT)/compat/config/%.c
	@mkdir -p $(OBJ)
	$(CC) $(PLAIN_CFLAGS) -MMD -MP -c $< -o $@

$(OBJ)/fh_%.o: $(GEN)/%.c | $(GEN)/qml_app.h
	@mkdir -p $(OBJ)
	$(CC) $(PLAIN_CFLAGS) -MMD -MP -c $< -o $@

$(OBJ)/fh_%.o: %.c | $(GEN)/conf/confparser.c
	@mkdir -p $(OBJ)
	$(CC) $(BASE_CFLAGS) -include stddef.h $(REFLOAT_INC) -MMD -MP -c $< -o $@

$(HOST_BIN): $(FH_OBJ)
	@mkdir -p $(BIN)
	$(CC) $^ -lm -lpthread -o $@

# --------------------------------------------------------------------- запуск

test: $(PROTO_TESTS_BIN) $(HOST_TESTS_BIN)
	@echo ""
	$(PROTO_TESTS_BIN)
	@echo ""
	$(HOST_TESTS_BIN)

# Интеграционный прогон поднимает FloatCore Host и говорит с ним по настоящему
# протоколу VESC — то же, что делает VESC Tool, только без GUI.
integration: $(HOST_BIN)
	$(ROOT)/tests/host_integration/run.sh

test-all: test integration

host: $(HOST_BIN)
	$(HOST_BIN) $(ARGS)

clean:
	rm -rf $(OBJ) $(BIN)/refloat_host_tests $(BIN)/protocol_tests $(BIN)/floatcore_host

-include $(OBJ)/*.d
