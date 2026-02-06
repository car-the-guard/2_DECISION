# =========================================================================
# D3-G Decision Board Build System
# Target: Raspberry Pi 4 (Aarch64) / TOPST D3-G
# =========================================================================

# [1] Toolchain (Cross Compiler)
# 우분투에서 D3-G(aarch64)용으로 빌드하기 위한 컴파일러
CC = aarch64-linux-gnu-gcc

# [2] Compiler Flags
# -Iinclude: 헤더 파일(.h) 위치 지정
# -pthread: 멀티스레드 사용
# -Wall: 모든 경고 출력 (코딩 실수 방지)
# -O2: 최적화 레벨 2
CFLAGS = -Wall -O2 -pthread -Iinclude

# [3] Linker Flags
# -pthread: 스레드 라이브러리 링크
# -lm: 수학 라이브러리 링크 (TTC 계산에서 sqrt 사용하므로 필수!)
LDFLAGS = -pthread -lm

# [4] Output Binary Name
# 생성될 실행 파일 이름
TARGET = app_main

# [5] Source Files
# 우리가 작성한 모든 모듈(.c)을 여기에 다 적어야 합니다.
SRC = \
    src/main.c \
    src/driving_info.c \
    src/collision_risk.c \
    src/collision_response.c \
    src/wl_sender.c \
    src/uart.c \
    src/can.c \
    src/bluetooth.c \
	src/can_security_utils.c \

# [6] Object Files (자동 생성)
# .c 파일 목록을 .o 파일 목록으로 자동 변환
OBJ = $(SRC:.c=.o)

# =========================================================================
# Build Rules
# =========================================================================

all: $(TARGET)

$(TARGET): $(OBJ)
	@echo "  [LINK] $@"
	$(CC) $(OBJ) -o $@ $(LDFLAGS)
	@echo "----------------------------------------"
	@echo "  Build Success! -> ./$(TARGET)"
	@echo "----------------------------------------"

# .c -> .o 컴파일 규칙
%.o: %.c
	@echo "  [CC]   $<"
	$(CC) $(CFLAGS) -c $< -o $@

# 청소 (make clean)
clean:
	@echo "  [CLEAN] Removing objects and binary..."
	rm -f $(OBJ) $(TARGET)

.PHONY: all clean