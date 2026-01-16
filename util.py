# vim: set ts=4 sw=4 sts=4 et :
import construct

PerAxis = lambda subcon: construct.Array(3, subcon)

# Use this in Structs for conditionally-present fields where we went to allow the user to
# skip those fields from the build() invocation dict when they know the condition is false.
# Basically we always want to do that for conditional fields in a Struct.
# The Default() allows the field to be skipped (it sets .flagbuildnone) but we still
# raise an error if the field should be present and was not provided by the user because
# the fallback value is None and it likely cannot be converted to the required type.
LazyIf = lambda cond, subcon: construct.Default(construct.If(cond, subcon), None)

class IntFlagsEnum(construct.FlagsEnum):
    def __init__(self, subcon, *merge, **mapping):
        super().__init__(subcon, *merge, **mapping)

        # Wrap the internal mapping with BitwiseInteger instances.
        for name, value in self.flags.items():
            wrapped = self._decode(value, None, name)
            setattr(self, name, wrapped)
    def _decode(self, obj, context, path):
        # Perform standard decoding into a Container of booleans
        decoded = super()._decode(obj, context, path)
        # Wrap it in our custom class that supports bitwise ops
        return BitwiseInteger(decoded, obj, self, context, path)
    def _build(self, obj, stream, context, path):
        # Building: ensure ctx[name] is a BitwiseInteger for use in If() and similar
        # 'obj' is the value passed to build (like a raw int or dict)
        raw_int = super()._encode(obj, context, path)
        wrapped = self._decode(raw_int, context, path)
        # We find our own name in the context to update it
        context[path.split('->')[-1].strip()] = wrapped
        # Continue with standard build using the raw integer
        return super()._build(raw_int, stream, context, path)

class BitwiseInteger(construct.Container):
    def __init__(self, container, raw_value, parent_enum, context, path):
        super().__init__(container)
        self._raw_value = raw_value
        self._parent_enum = parent_enum
        self._context = context
        self._path = path
    def _wrap_int(self, new_val):
        return self._parent_enum._decode(new_val, self._context, self._path)

    def __int__(self): return self._raw_value
    def __and__(self, other):    return self._raw_value & int(other)
    def __rand__(self, other):   return int(other) & self._raw_value
    def __or__(self, other):     return self._raw_value | int(other)
    def __xor__(self, other):    return self._raw_value ^ int(other)
    def __lshift__(self, other): return self._wrap_int(self._raw_value << int(other))
    def __rshift__(self, other): return self._wrap_int(self._raw_value >> int(other))

    def __repr__(self):
        return f"BitwiseInteger({self._raw_value}, {super().__repr__()})"

class FlatRawCopy(construct.RawCopy):
    def _build(self, obj, stream, context, path):
        # Accept inner value directly
        return super()._build(construct.Container(value=obj), stream, context, path)

    def _parse(self, stream, context, path):
        result = super()._parse(stream, context, path)
        # Flatten: merge value fields into the container
        if isinstance(result.value, bytes):
            flat = construct.Container()
        else:
            flat = construct.Container(result.value)
        flat.data = result.data
        flat.offset1 = result.offset1
        flat.offset2 = result.offset2
        return flat
