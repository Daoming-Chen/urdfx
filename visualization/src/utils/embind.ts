import type { EmbindVector } from '../../public/urdfx';

/**
 * Converts embind vector instances to plain JavaScript arrays so downstream code
 * can rely on standard array helpers.
 */
export function embindVectorToArray<T>(
  source: EmbindVector<T> | ArrayLike<T> | T[] | null | undefined,
): T[] {
  if (!source) {
    return [];
  }

  if (Array.isArray(source)) {
    return [...source];
  }

  if (ArrayBuffer.isView(source)) {
    return Array.from(source as ArrayLike<T>);
  }

  if (isEmbindVector(source)) {
    const size = source.size();
    const result = new Array<T>(size);
    for (let i = 0; i < size; i += 1) {
      result[i] = source.get(i);
    }
    source.delete?.();
    return result;
  }

  return Array.from(source as ArrayLike<T>);
}

function isEmbindVector<T>(value: unknown): value is EmbindVector<T> {
  if (!value || typeof value !== 'object') {
    return false;
  }

  const candidate = value as Partial<EmbindVector<T>>;
  return typeof candidate.size === 'function' && typeof candidate.get === 'function';
}
