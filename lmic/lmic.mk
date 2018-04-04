### Variables
LMIC_DIR = $(APP_DIR)/lmic
OBJ_DIRS += $(LMIC_DIR)/$(BUILD)/$(SOC)/$(TARGET)/$(OBJ)

LMIC_SOURCES =
LMIC_SOURCES += lmic.c
LMIC_SOURCES += oslmic.c
LMIC_SOURCES += radio.c
LMIC_SOURCES += hal/hal.c
LMIC_SOURCES += aes/lmic.c
LMIC_SOURCES += aes/other.c
LMIC_SOURCES += aes/ideetron/AES-128_V10.c

# Objects
OBJECTS += $(addprefix $(LMIC_DIR)/$(BUILD)/$(SOC)/$(TARGET)/$(OBJ)/,$(LMIC_SOURCES:.c=.o))
GENERATED_DIRS += $(LMIC_DIR)/$(BUILD)

### Flags
# CFLAGS += -I$(LMIC_DIR)

$(LMIC_DIR)/$(BUILD)/$(SOC)/$(TARGET)/$(OBJ)/%.o: $(LMIC_DIR)/%.c
	$(call mkdir, $(LMIC_DIR)/$(BUILD)/$(SOC)/$(TARGET)/$(OBJ))
	$(call mkdir, $(LMIC_DIR)/$(BUILD)/$(SOC)/$(TARGET)/$(OBJ)/hal)
	$(call mkdir, $(LMIC_DIR)/$(BUILD)/$(SOC)/$(TARGET)/$(OBJ)/aes)
	$(call mkdir, $(LMIC_DIR)/$(BUILD)/$(SOC)/$(TARGET)/$(OBJ)/aes/ideetron)
	$(CC) $(CFLAGS) -c -o $@ $<