ifndef _version_mk_
_version_mk_ = 1


ifndef VERSION
VERSION := $(shell sh version.sh . 2>/dev/null)
endif
REVISION_STR := $(word 1, $(VERSION))
REVISION_SGN := $(word 2, $(VERSION))

ifdef V
$(info revision string: $(REVISION_STR), signature: $(REVISION_SGN))
endif

COMMON_CFLAGS += -DREVISION_STR=\"$(REVISION_STR)\" -DREVISION_SGN=$(REVISION_SGN)
COMMON_CFLAGS += -DPROJECT_NAME=\"$(TARGET)\"

endif