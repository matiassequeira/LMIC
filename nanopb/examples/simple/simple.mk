SIMPLE_DIR := $(NANOPB_DIR)/examples/simple
$(info SIMPLE_DIR = $(SIMPLE_DIR))

SIMPLE_SOURCES = $(wildcard $(SIMPLE_DIR)/*.c)
OBJ_DIRS += $(SIMPLE_DIR)/$(BUILD)/$(SOC)/$(OBJ)
OBJECTS += $(addprefix $(SIMPLE_DIR)/$(BUILD)/$(SOC)/$(OBJ)/,$(notdir $(SIMPLE_SOURCES:.c=.o)))
CFLAGS += -I$(SIMPLE_DIR)

$(SIMPLE_DIR)/$(BUILD)/$(SOC)/$(OBJ)/%.o: $(SIMPLE_DIR)/%.c
	$(call mkdir, $(SIMPLE_DIR)/$(BUILD)/$(SOC)/$(OBJ))
	$(CC) $(CFLAGS) -c -o $@ $<