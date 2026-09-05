package org.example.export;

import java.io.BufferedWriter;
import java.io.Closeable;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.OpenOption;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.List;

/**
 * Экспорт облака точек в ASCII-формат XYZ: одна строка "x y z" на точку.
 * Универсальный fallback: открывается везде (CloudCompare, SolidWorks, open3d...),
 * в отличие от PLY/PCD.
 *
 * <p>Два режима:
 * <ul>
 *  <li>{@link #write(Path, List)} — одноразовая выгрузка готового облака;</li>
 *  <li>{@link #open(Path)} — {@link Stream} для инкрементальной записи
 *      (свип: точка дописывается по мере накопления, файл виден "на лету").</li>
 * </ul>
 */
public final class XyzWriter {

    private XyzWriter() {
    }

    public static void write(Path file, List<float[]> points) throws IOException {
        if (points == null) {
            throw new IllegalArgumentException("points == null");
        }
        try (Stream s = open(file)) {
            for (float[] p : points) {
                s.writePoint(p);
            }
        }
    }

    /**
     * Открывает файл на инкрементальную запись (создаёт/обрезает), UTF-8,
     * строки разделяются '\n' (не newLine(): на Windows newLine() даёт "\r\n").
     */
    public static Stream open(Path file, OpenOption... options) throws IOException {
        OpenOption[] opts = options.length == 0
                ? new OpenOption[]{StandardOpenOption.CREATE, StandardOpenOption.TRUNCATE_EXISTING}
                : options;
        return new Stream(new BufferedWriter(new OutputStreamWriter(Files.newOutputStream(file, opts), StandardCharsets.UTF_8)));
    }

    /**
     * Инкрементальный писатель: {@link #writePoint(float[])} дописывает строку,
     * {@link #count()} — сколько точек записано. Не блокирующий: буферизует
     * и пишет по мере накопления.
     */
    public static final class Stream implements Closeable {
        private final BufferedWriter writer;
        private long count;

        private Stream(BufferedWriter writer) {
            this.writer = writer;
        }

        public void writePoint(float[] p) throws IOException {
            writer.write(String.valueOf(p[0]));
            writer.write(' ');
            writer.write(String.valueOf(p[1]));
            writer.write(' ');
            writer.write(String.valueOf(p[2]));
            writer.write('\n');
            count++;
        }

        public long count() {
            return count;
        }

        @Override
        public void close() throws IOException {
            writer.flush();
            writer.close();
        }
    }
}
