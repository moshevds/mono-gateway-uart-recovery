#!/bin/sh
set -eu

cd "$(dirname "$0")"

cargo build \
    -p mono-uart-recovery-stub \
    --bin mono-uart-recovery-stub \
    --release \
    --target aarch64-unknown-none

cargo rustc \
    -p mono-uart-recovery-stub \
    --bin mono-uart-recovery-stage1 \
    --release \
    --target aarch64-unknown-none \
    -- \
    -C link-arg=--defsym=__ocram_origin=0x10004000 \
    -C link-arg=--defsym=__ocram_length=0x1c000

RUST_HOST="$(rustc -vV | sed -n 's/^host: //p')"
RUST_OBJCOPY="$(rustc --print sysroot)/lib/rustlib/$RUST_HOST/bin/rust-objcopy"

if command -v rust-objcopy >/dev/null 2>&1; then
    OBJCOPY="$(command -v rust-objcopy)"
elif [ -x "$RUST_OBJCOPY" ]; then
    OBJCOPY="$RUST_OBJCOPY"
else
    OBJCOPY=objcopy
fi

"$OBJCOPY" \
    -O binary \
    target/aarch64-unknown-none/release/mono-uart-recovery-stub \
    target/aarch64-unknown-none/release/mono-uart-recovery-stub.bin

"$OBJCOPY" \
    -O binary \
    target/aarch64-unknown-none/release/mono-uart-recovery-stage1 \
    target/aarch64-unknown-none/release/mono-uart-recovery-stage1.bin

ls -lh \
    target/aarch64-unknown-none/release/mono-uart-recovery-stub.bin \
    target/aarch64-unknown-none/release/mono-uart-recovery-stage1.bin
