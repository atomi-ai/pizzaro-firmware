# Lib Structure for future

## Why do we need such a Library `lib/`?

For our embedded projects like this, some code should be independent of std, and such libraries can be placed in `lib/`; some code needs to be no_std, and we can consider placing it in directories like `hal_lib/` or `embedded_lib/`, with the specific names still needing further discussion.

The benefit of this is that we have many unit tests that can conform to Rust standards, written directly in the corresponding code, making encapsulation and other aspects much more convenient."

## Run tests and coverage

Install tarpaulin
```shell
cargo install cargo-llvm-cov
```

Run test and coverage
```shell
cargo test --target x86_64-unknown-linux-gnu --packages common --out Html --output-dir /tmp/coverage/
cargo llvm-cov --target x86_64-unknown-linux-gnu --package common --html
```


