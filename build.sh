#!/usr/bin/env bash

# Kernel
DEFCONFIG=vendor/sm6115_defconfig
IMAGE=$(pwd)/out/arch/arm64/boot/Image

# Architecture
export ARCH=arm64
export SUBARCH=arm64

# Build information
export KBUILD_BUILD_HOST=frostedarch
export KBUILD_BUILD_USER="shiina"

# Device information
export CODENAME="SM6115"
export DEVICE="Xiaomi SM6115 platform"

# Function environment
function post_msg() {
	curl -s -X POST "https://api.telegram.org/bot$TOKEN/sendMessage" \
	-d chat_id="$CHAT_ID" \
	-d "disable_web_page_preview=true" \
	-d "parse_mode=html" \
	-d text="$1"
	}

function push() {
	curl -F document=@$1 "https://api.telegram.org/bot$TOKEN/sendDocument" \
	-F chat_id="$CHAT_ID"
	}

# Clone Yuki clang 17.0.0 (if clang not exist)
if ! [ -a "$(pwd)/clang" ]; then
	git clone --depth=1 https://gitlab.com/klozz/yuki-clang-new.git -b 17.0.0 clang
fi

# Export clang path to PATH env
PATH="$(pwd)/clang/bin:$PATH"

# Export GCC path to PATH env (if exists)
if [ -a "$(pwd)/gcc*" ]; then
	PATH="$(pwd)/gcc64/bin:$(pwd)/gcc32/bin:$PATH"
fi

# Notify kernel compilation started
post_msg "<b>Katana Kernel Compilation Started</b>%0Aby the <b>frostedscape.</b> org%0A%0A<b>Host OS: </b><code>$(source /etc/os-release && echo "${NAME}")</code>%0A<b>Kernel version: </b><code>$(make kernelversion)</code>%0A<b>Build date: </b><code>$(TZ=Asia/Jakarta date)</code>%0A<b>Device: </b><code>$DEVICE [$CODENAME]</code>%0A<b>Hostname: </b><code>$KBUILD_BUILD_HOST</code>%0A<b>Host core count: </b><code>$(nproc --all)</code>%0A<b>Compiler used: </b><code>$($(pwd)/clang/bin/clang --version | head -n 1 | perl -pe 's/\(http.*?\)//gs' | sed -e 's/  */ /g' -e 's/[[:space:]]*$//')</code>%0A<b>Branch: </b><code>$(git rev-parse --abbrev-ref HEAD)</code>%0A<b>Last commit: </b><code>$(git log --pretty=format:'%h: %s' -1)</code>%0A%0A#katana #$CODENAME"

# Cleanup before compiling
rm -f AnyKernel3/Image
rm -f AnyKernel3/*.zip
rm -f out/build_log.txt

# Start compilation
make O=out ARCH=arm64 CC=clang $DEFCONFIG
make -j$(nproc --all) \
	O=out \
    ARCH=arm64 \
    CC=clang \
    CLANG_TRIPLE=aarch64-linux-gnu- \
    CROSS_COMPILE=aarch64-linux-gnu- \
    CROSS_COMPILE_ARM32=arm-linux-gnueabi- \
    2>&1 | tee out/build_log.txt

if ! [ -a "$IMAGE" ]; then
	post_msg "Compile failed. Check build log below!!!"
	push "out/build_log.txt"
	exit 1
fi

# Clone AnyKernel3
git clone --depth=1 https://github.com/frstprjkt/AnyKernel3 -b SM6115
rm -rf AnyKernel3/{.git,.github}

# Move kernel files into AnyKernel3
mv $IMAGE AnyKernel3

# Compress AnyKernel3 into zip
cd AnyKernel3
zip -r9 katana-$CODENAME-$(date +%d%H%M).zip .
cd ..

# Upload kernel zip
push "$(ls AnyKernel3/katana*)"
push "out/build_log.txt"
