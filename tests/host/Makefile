# Совместимость с прежним путём сборки: всё собирается корневым Makefile.
.PHONY: all test clean gen
all test clean gen:
	$(MAKE) -C ../.. $(if $(filter test,$@),test,$(if $(filter clean,$@),clean,$(if $(filter gen,$@),gen,host-tests)))
