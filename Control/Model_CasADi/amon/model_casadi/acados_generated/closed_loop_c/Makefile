GENERATED_DIR := ../c_generated_code
ACADOS_DIR := ../../../acados

CC ?= cc

CPPFLAGS += -I$(GENERATED_DIR)
CPPFLAGS += -I$(ACADOS_DIR)/include
CPPFLAGS += -I$(ACADOS_DIR)/include/acados
CPPFLAGS += -I$(ACADOS_DIR)/include/blasfeo/include
CPPFLAGS += -I$(ACADOS_DIR)/include/hpipm/include

CFLAGS += -fPIC -std=c99 -O2
LDFLAGS += -L$(ACADOS_DIR)/lib
LDLIBS += -lacados -lhpipm -lblasfeo -lm

MODEL_SRC := \
	$(GENERATED_DIR)/amon_model_model/amon_model_expl_ode_fun.c \
	$(GENERATED_DIR)/amon_model_model/amon_model_expl_vde_forw.c \
	$(GENERATED_DIR)/amon_model_model/amon_model_expl_vde_adj.c

SOLVER_SRC := \
	$(GENERATED_DIR)/acados_solver_amon_model.c \
	$(GENERATED_DIR)/acados_sim_solver_amon_model.c

MAIN_SRC := main_closed_loop_amon_model.c

EXE := main_closed_loop_amon_model

.PHONY: all clean run

all: $(EXE)

$(EXE): $(MAIN_SRC) $(MODEL_SRC) $(SOLVER_SRC)
	$(CC) $(CPPFLAGS) $(CFLAGS) $^ -o $@ $(LDFLAGS) $(LDLIBS)

run: $(EXE)
	./$(EXE)

clean:
	$(RM) $(EXE)
