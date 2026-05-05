#if !NET5_0_OR_GREATER
namespace System
{
    internal readonly struct Index
    {
        private readonly int _value;
        public Index(int value, bool fromEnd = false)
            => _value = fromEnd ? ~value : value;

        public static Index FromEnd(int value) => new Index(value, true);
        public bool IsFromEnd => _value < 0;
        public int Value => IsFromEnd ? ~_value : _value;

        public int GetOffset(int length)
        {
            int offset = _value;
            if (IsFromEnd) offset += length + 1;
            return offset;
        }

        public static implicit operator Index(int value) => new Index(value);
        public override string ToString() => IsFromEnd ? $"^{Value}" : Value.ToString();
    }

    internal readonly struct Range
    {
        public Index Start { get; }
        public Index End { get; }

        public Range(Index start, Index end) { Start = start; End = end; }
        public static Range All => new Range(Index.FromEnd(0), Index.FromEnd(0));

        public (int Offset, int Length) GetOffsetAndLength(int length)
        {
            int start = Start.GetOffset(length);
            int end = End.GetOffset(length);
            return (start, end - start);
        }
    }
}
#endif