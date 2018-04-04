ESLIGHT_DIR := $(APP_DIR)/eslight
$(info ESLIGHT_DIR = $(ESLIGHT_DIR))

ESLIGHT_SOURCES = $(wildcard $(ESLIGHT_DIR)/*.c)
OBJ_DIRS += $(ESLIGHT_DIR)/$(BUILD)/$(SOC)/$(OBJ)
OBJECTS += $(addprefix $(ESLIGHT_DIR)/$(BUILD)/$(SOC)/$(OBJ)/,$(notdir $(ESLIGHT_SOURCES:.c=.o)))

$(ESLIGHT_DIR)/$(BUILD)/$(SOC)/$(OBJ)/%.o: $(ESLIGHT_DIR)/%.c $(ESLIGHT_DIR)/%.h
	$(call mkdir, $(ESLIGHT_DIR)/$(BUILD)/$(SOC)/$(OBJ))
	$(CC) $(CFLAGS) -c -o $@ $<