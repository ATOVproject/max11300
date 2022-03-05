```rust
struct Alpha;
struct Beta;
struct Gamma;
struct Delta;

// Pretend I've done the sealed trait thing with the above types...


struct Container<N>
// where N: ...
{
    mode: N,
}

impl<N> Container<N> {
    fn to_alpha(self) -> Container<Alpha> {
        // ...
    }
    
    fn to_beta(self) -> Container<Beta> {
        // ...
    }
    
    fn to_gamma(self) -> Container<Gamma> {
        // ...
    }
    
    fn to_delta(self) -> Container<Delta> {
        // ...
    }
}

impl Container<Gamma> {
    fn do_gamma_things(&mut self, data: &[u8]) {}
}

// Marker trait
trait InputCapable {}
impl InputCapable for Alpha {}
impl InputCapable for Beta {}

impl<N: InputCapable> Container<N> {
    fn get_input(&mut self) -> u16 {}
}
```

```rust
struct Alpha;
struct Beta;
struct Gamma;
struct Delta;

// Pretend I've done the sealed trait thing with the above types...


struct Container<N>
// where N: ...
{
    mode: N,
    port: SPI,
}

impl<N> Container<N> {
    fn to_alpha(self) -> Container<Alpha> {
        // ...
        do_expensive_thing(&mut self.spi, [0x01, 0x02]);
    }
    
    fn to_beta(self) -> Container<Beta> {
        // ...
        do_expensive_thing(&mut self.spi, [0x02, 0x03]);
    }
    
    fn to_gamma(self) -> Container<Gamma> {
        // ...
        do_expensive_thing(&mut self.spi, [0x03, 0x04]);
    }
    
    fn to_delta(self) -> Container<Delta> {
        // ...
        do_expensive_thing(&mut self.spi, [0x04, 0x05]);
    }
}

#[inline(never)]
fn do_expensive_thing<SPI>(spi: &mut SPI) {
    
}
```

Reduce "monomorphization" bloat

for code size bloat it often helps to implement the logic in non-generic, like this

```rust
fn read(&mut self, channel: u32) -> u32;
fn set_state(&mut self, channel: u32, mode: Mode);
```

and then build the public generic typestate API on top of that
