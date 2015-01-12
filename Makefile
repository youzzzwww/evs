# GNU Makefile

# Paths
SRC_ENC   = lib_enc lib_com basic_op basic_math
SRC_DEC   = lib_dec lib_com basic_op basic_math
BUILD     = build

SRC_DIRS  = $(sort -u $(SRC_ENC) $(SRC_DEC)) 

# Name of CLI binaries
CLI_ENC   = EVS_cod
CLI_DEC   = EVS_dec

# Default tool settings
CC        = gcc
RM        = rm -f

ifndef VERBOSE
QUIET_CC  = @echo '   ' Compiling $<;
QUIET_LINK= @echo '   ' Linking $@;
QUIET     = @
endif

# C compiler flags
CFLAGS   += -pedantic -Wcast-qual -Wall -W -Wextra -Wno-long-long     \
            -Wpointer-arith -Wstrict-prototypes -Wmissing-prototypes  \
            -Werror-implicit-function-declaration 

ifeq "$(RELEASE)" "1"
CFLAGS   += -DRELEASE
DELIVERY  = 1
OPTIM    ?= 2
endif

ifneq "$(DEBUG)" "0"
CFLAGS   += -g3
LDFLAGS  += -g3
endif

ifeq "$(WMOPS)" "1"
CFLAGS   += -DWMOPS=1
endif

OPTIM    ?= 0
CFLAGS   += -O$(OPTIM)

CFLAGS   += $(foreach DIR,$(SRC_DIRS),-I$(DIR))

# Source file search paths
VPATH     = $(SRC_DIRS)

###############################################################################

SRCS_ENC  = $(foreach DIR,$(SRC_ENC),$(patsubst $(DIR)/%,%,$(wildcard $(DIR)/*.c)))
SRCS_DEC  = $(foreach DIR,$(SRC_DEC),$(patsubst $(DIR)/%,%,$(wildcard $(DIR)/*.c)))

OBJS_ENC  = $(addprefix $(BUILD)/,$(SRCS_ENC:.c=.o))
OBJS_DEC  = $(addprefix $(BUILD)/,$(SRCS_DEC:.c=.o))

DEPS      = $(addprefix $(BUILD)/,$(SRCS_ENC:.c=.P) $(SRCS_DEC:.c=.P))

###############################################################################

.PHONY: all clean clean_all

all: $(CLI_ENC) $(CLI_DEC)

$(BUILD):
	$(QUIET)mkdir -p $(BUILD)

$(CLI_ENC): $(OBJS_ENC)
	$(QUIET_LINK)$(CC) $(LDFLAGS) $(OBJS_ENC) -lm -o $(CLI_ENC)

$(CLI_DEC): $(OBJS_DEC)
	$(QUIET_LINK)$(CC) $(LDFLAGS) $(OBJS_DEC) -lm -o $(CLI_DEC)

clean:
	$(QUIET)$(RM) $(OBJS_ENC) $(OBJS_DEC) $(DEPS)
	$(QUIET)$(RM) $(DEPS:.P=.d)
	$(QUIET)test ! -d $(BUILD) || rm -rf $(BUILD)

clean_all: clean
	$(QUIET)$(RM) $(CLI_ENC) $(CLI_DEC)

$(BUILD)/%.o : %.c | $(BUILD)
	$(QUIET_CC)$(CC) $(CFLAGS) -c -MD -o $@ $<
	@cp $(BUILD)/$*.d $(BUILD)/$*.P; \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
	    -e '/^$$/ d' -e 's/$$/ :/' < $(BUILD)/$*.d >> $(BUILD)/$*.P; \
	$(RM) $(BUILD)/$*.d

-include $(DEPS)
