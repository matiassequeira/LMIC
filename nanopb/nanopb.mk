NANOPB_DIR := $(APP_DIR)/nanopb
$(info NANOPB_DIR = $(NANOPB_DIR))

NANOPB_SOURCES = $(wildcard $(NANOPB_DIR)/*.c)
OBJ_DIRS += $(NANOPB_DIR)/$(BUILD)/$(SOC)/$(OBJ)
OBJECTS += $(addprefix $(NANOPB_DIR)/$(BUILD)/$(SOC)/$(OBJ)/,$(notdir $(NANOPB_SOURCES:.c=.o)))
CFLAGS += -I$(NANOPB_DIR)

$(NANOPB_DIR)/$(BUILD)/$(SOC)/$(OBJ)/%.o: $(NANOPB_DIR)/%.c $(NANOPB_DIR)/%.h
	$(call mkdir, $(NANOPB_DIR)/$(BUILD)/$(SOC)/$(OBJ))
	$(CC) $(CFLAGS) -c -o $@ $<

# Uncomment this line to build the example
# include $(NANOPB_DIR)/examples/simple/simple.mk