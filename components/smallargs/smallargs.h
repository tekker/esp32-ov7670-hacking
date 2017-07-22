/*
 * smallargs.h
 *
 *  Created on: 6 Oct 2016
 *      Author: Fabian Meyer
 *
 * LICENSE
 * =======
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Fabian Meyer
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef INCLUDE_SMALLARGS_H_
#define INCLUDE_SMALLARGS_H_

#include <stddef.h>
#include <string.h>
#include <stdlib.h>

#define SARG_VERSION "0.1.0"

#define SARG_ERR_SUCCESS       0
#define SARG_ERR_ERRNO        -1
#define SARG_ERR_OTHER        -2
#define SARG_ERR_INVALARG     -3
#define SARG_ERR_PARSE        -4
#define SARG_ERR_NOTFOUND     -5
#define SARG_ERR_ALLOC        -6
#define SARG_ERR_ABORT        -7

#define _SARG_UNUSED(e) ((void) e)
#define _SARG_IS_SHORT_ARG(s) (s[0] == '-' && s[1] != '-')
#define _SARG_IS_LONG_ARG(s) (s[0] == '-' && s[1] == '-')
#define _SARG_IS_HEX_NUM(s) (s[0] == '0' && s[1] == 'x')
#define _SARG_IS_OCT_NUM(s) (s[0] == '0' && strchr("1234567", s[1]) != NULL)


#define sarg_foreach(root, it) \
    for(_sarg_iterator_init((it), (root)); \
        _sarg_iterator_has_next((it)); \
        _sarg_iterator_next((it)))

typedef enum _sarg_opt_type {
    INT = 0,
    UINT,
    DOUBLE,
    BOOL,
    STRING,
    COUNT
} sarg_opt_type;

typedef struct _sarg_result {
    sarg_opt_type type;
    int count;
    union {
        int int_val;
        unsigned int uint_val;
        double double_val;
        int bool_val;
        char *str_val;
    };
} sarg_result;

typedef int (*sarg_opt_cb)(const sarg_result *);

typedef struct _sarg_opt {
    char *short_name;
    char *long_name;
    char *help;
    sarg_opt_type type;
    sarg_opt_cb callback;
} sarg_opt;

typedef struct _sarg_root {
    char *name;
    sarg_opt *opts;
    int opt_len;
    sarg_result *results;
    int res_len;
} sarg_root;

typedef struct _sarg_iterator {
    sarg_root *root;
    sarg_result *result;
    char *name;
    int idx;
} sarg_iterator;

void _sarg_iterator_init(sarg_iterator *it, sarg_root *root)
{
    it->root = root;
    it->result = NULL;
    it->name = NULL;
    it->idx = 0;

    if(root->opt_len > 0)
    {
        it->result = &root->results[0];
        it->name = root->opts[0].short_name != NULL ?
            root->opts[0].short_name : root->opts[0].short_name;
    }
}

int _sarg_iterator_has_next(sarg_iterator *it)
{
    return it->idx < it->root->opt_len;
}

void _sarg_iterator_next(sarg_iterator *it)
{
    int i = ++(it->idx);
    if(i < it->root->opt_len)
    {
        it->result = &it->root->results[i];
        it->name = it->root->opts[i].short_name != NULL ?
            it->root->opts[i].short_name : it->root->opts[i].long_name;
    }
}

const char * sarg_strerror(const int errval)
{
    _SARG_UNUSED(errval);
    return "small args error";
}

void _sarg_result_destroy(sarg_result *res)
{
    if(res->type == STRING && res->str_val) {
        free(res->str_val);
        res->str_val = NULL;
    }
}

void _sarg_opt_destroy(sarg_opt *opt)
{
    if(opt->short_name)
        free(opt->short_name);
    if(opt->long_name)
        free(opt->long_name);
    if(opt->help)
        free(opt->help);
}

/**
 * @brief Destroys the given root and frees its memory.
 *
 * @param root root object that will be cleared
 */
void sarg_destroy(sarg_root *root)
{
    int i;
    if(root->results) {
        for(i = 0; i < root->res_len; ++i)
            _sarg_result_destroy(&root->results[i]);
        free(root->results);
    }

    root->results = NULL;
    root->res_len = -1;

    if(root->opts) {
        for(i = 0; i < root->opt_len; ++i)
            _sarg_opt_destroy(&root->opts[i]);
        free(root->opts);
    }
    root->opts = NULL;
    root->opt_len = -1;

    if(root->name)
        free(root->name);
    root->name = NULL;
}

void _sarg_result_init(sarg_result *res, sarg_opt_type type)
{
    memset(res, 0, sizeof(sarg_result));
    res->type = type;
}

int _sarg_opt_init(sarg_opt *opt, const char *short_name, const char *long_name, const char *help, const sarg_opt_type type, sarg_opt_cb cb)
{
    memset(opt, 0, sizeof(sarg_opt));

    if(short_name) {
        opt->short_name = (char *) malloc(strlen(short_name) + 1);
       if(!opt->short_name)
           return SARG_ERR_ALLOC;
       strcpy(opt->short_name, short_name);
    }

    if(long_name) {
        opt->long_name = (char *) malloc(strlen(long_name) + 1);
        if(!opt->long_name)
            return SARG_ERR_ALLOC;
        strcpy(opt->long_name, long_name);
    }

    if(help) {
        opt->help = (char *) malloc(strlen(help) + 1);
        if(!opt->help)
            return SARG_ERR_ALLOC;
        strcpy(opt->help, help);
    }

    opt->callback = cb;
    opt->type = type;

    return SARG_ERR_SUCCESS;
}

int _sarg_opt_duplicate(sarg_opt *dest, const sarg_opt *src)
{
    return _sarg_opt_init(dest, src->short_name, src->long_name, src->help, src->type, src->callback);
}

int _sarg_opt_len(const sarg_opt *options)
{
    int i;

    for(i = 0; options[i].short_name || options[i].long_name; ++i);

    return i;
}

/**
 * @brief Initializes the root data structure with the given options.
 *
 * @param root root data structure which will be used to parse arguments
 * @param options NULL-terminated array of allowed options
 * @param name name of the application
 *
 * @return SARG_ERR_SUCCESS on success or a SARG_ERR_* code otherwise
 */
int sarg_init(sarg_root *root, const sarg_opt *options, const char *name)
{
    int i, ret, len;

    memset(root, 0, sizeof(sarg_root));

    len = _sarg_opt_len(options);

    // init option array
    root->opts = (sarg_opt *) malloc(sizeof(sarg_opt) * len);
    if(!root->opts)
        return SARG_ERR_ALLOC;

    memset(root->opts, 0, sizeof(sarg_opt) * len);
    root->opt_len = len;

    // init result array
    root->results = (sarg_result *) malloc(sizeof(sarg_result) * len);
    if(!root->results) {
        sarg_destroy(root);
        return SARG_ERR_ALLOC;
    }

    memset(root->results, 0, sizeof(sarg_result) * len);
    root->res_len = len;

    // init name
    root->name = (char *) malloc(strlen(name) + 1);
    if(!root->name) {
        sarg_destroy(root);
        return SARG_ERR_ALLOC;
    }
    strcpy(root->name, name);

    // duplicate the given options
    for(i = 0; i < root->opt_len; ++i) {
        ret = _sarg_opt_duplicate(&root->opts[i], &options[i]);
        if(ret != SARG_ERR_SUCCESS) {
            sarg_destroy(root);
            return ret;
        }
    }

    // init results
    for(i = 0; i < root->res_len; ++i)
        _sarg_result_init(&root->results[i], root->opts[i].type);

    return SARG_ERR_SUCCESS;
}

int _sarg_find_opt(sarg_root *root, const char *name)
{
    int i, is_short, is_long;
    char *namep = (char *) name;

    while(namep[0] == '-')
        ++namep;

    for(i = 0; i < root->opt_len; ++i) {
        is_short = root->opts[i].short_name
                   && strcmp(namep, root->opts[i].short_name) == 0;
        is_long = root->opts[i].long_name
                  && strcmp(namep, root->opts[i].long_name) == 0;
        if(is_short || is_long)
            return i;
    }

    return -1;
}

int _sarg_get_number_base(const char *arg)
{
    int len = strlen(arg);
    int base = 10;

    // define base for number conversion
    if(len > 2) {
        if(_SARG_IS_OCT_NUM(arg))
            base = 8;
        if(_SARG_IS_HEX_NUM(arg))
            base = 16;
    }

    return base;
}

int _sarg_parse_int(const char *arg, sarg_result *res)
{
    int base;
    char *endptr;
    base = _sarg_get_number_base(arg);

    // convert and check if conversion succeeded
    res->int_val = strtol(arg, &endptr, base);
    if(*endptr != '\0')
        return SARG_ERR_PARSE;

    return SARG_ERR_SUCCESS;
}

int _sarg_parse_uint(const char *arg, sarg_result *res)
{
    int base;
    char *endptr;
    base = _sarg_get_number_base(arg);

    // convert and check if conversion succeeded
    res->uint_val = strtoul(arg, &endptr, base);
    if(*endptr != '\0')
        return SARG_ERR_PARSE;

    return SARG_ERR_SUCCESS;
}

int _sarg_parse_double(const char *arg, sarg_result *res)
{
    char *endptr;

    res->double_val = strtod(arg, &endptr);
    if(*endptr != '\0')
        return SARG_ERR_PARSE;

    return SARG_ERR_SUCCESS;
}

int _sarg_parse_bool(const char *arg, sarg_result *res)
{
    _SARG_UNUSED(arg);
    res->bool_val = !res->bool_val;

    return SARG_ERR_SUCCESS;
}

int _sarg_parse_str(const char *arg, sarg_result *res)
{
    if(res->str_val)
        free(res->str_val);

    res->str_val = (char*) malloc(strlen(arg) + 1);
    if(!res->str_val)
        return SARG_ERR_ALLOC;

    strcpy(res->str_val, arg);

    return SARG_ERR_SUCCESS;
}

typedef int (*_sarg_parse_func)(const char *, sarg_result *);
static _sarg_parse_func _sarg_parse_funcs[COUNT] = {
    _sarg_parse_int,
    _sarg_parse_uint,
    _sarg_parse_double,
    _sarg_parse_bool,
    _sarg_parse_str,
};

/**
 * @brief Parses the given arguments with the given root object.
 *
 * The root object has to be initialized with sarg_init before
 * being passed to this function.
 *
 * If specified this function will call callback functions on the
 * appearance of the corresponding options.
 *
 * @param root root object which should be used to parse arguments
 * @param argv array of arguments to be parsed
 * @param argc number of elements in argv
 *
 * @return SARG_ERR_SUCCESS on success or a SARG_ERR_* code otherwise
 */
int sarg_parse(sarg_root *root, const char **argv, const int argc)
{
    int i, arg_idx, len, ret;

    for(i = 1; i < argc; ++i) {
        // argument has to have at least 2 chars
        len = strlen(argv[i]);
        if(len < 2)
            return SARG_ERR_PARSE;

        if(_SARG_IS_SHORT_ARG(argv[i]) || _SARG_IS_LONG_ARG(argv[i])) {
            // find option
            arg_idx = _sarg_find_opt(root, argv[i]);
            if(arg_idx < 0)
                return SARG_ERR_NOTFOUND;

            if(root->opts[arg_idx].type != BOOL) {
                ++i;
                if(i >= argc)
                    return SARG_ERR_PARSE;
            }

            ret = _sarg_parse_funcs[root->opts[arg_idx].type](
                      argv[i], &root->results[arg_idx]);
            if(ret != SARG_ERR_SUCCESS)
                return ret;

            ++root->results[arg_idx].count;

            // call callback if it was set
            if(root->opts[arg_idx].callback) {
                ret = root->opts[arg_idx].callback(&root->results[arg_idx]);
                if(ret != SARG_ERR_SUCCESS)
                    return ret;
            }
        }
    }

    return SARG_ERR_SUCCESS;
}

/**
 * @brief Access the parsing result of the specified option.
 *
 * @param root root object that was used to parse arguments
 * @param name short or long name of the option
 * @param res result object for the given option
 *
 * @return SARG_ERR_SUCCESS on success or SARG_ERR_NOTFOUND if the option was not found
 */
int sarg_get(sarg_root *root, const char *name, sarg_result **res)
{
    int arg_idx;

    arg_idx = _sarg_find_opt(root, name);
    if(arg_idx < 0)
        return SARG_ERR_NOTFOUND;

    *res = &root->results[arg_idx];

    return SARG_ERR_SUCCESS;
}

#ifndef SARG_NO_PRINT

#include <stdarg.h>
#include <stdio.h>

int _sarg_buf_resize(char **buf, int *len)
{
    char *buf_tmp = *buf;
    int old_len = *len;

    *buf = (char *) malloc(old_len * 2);
    if(!*buf) {
        *buf = buf_tmp;
        return SARG_ERR_ALLOC;
    }

    memcpy(*buf, buf_tmp, old_len);
    *len = 2 * old_len;

    free(buf_tmp);

    return SARG_ERR_SUCCESS;
}

int _sarg_snprintf(char **buf, int *len, int *off, const char *fmt, ...)
{
    va_list args;
    int ret, write_len;
    char *curr_ptr;

    while(1) {
        write_len = *len - *off;
        curr_ptr = &((*buf)[*off]);

        va_start(args, fmt);

        ret = vsnprintf(curr_ptr, write_len, fmt, args);

        va_end(args);

        if(ret < 0)
            return SARG_ERR_OTHER;

        if(ret < write_len) {
            *off += ret;
            break;
        }

        ret = _sarg_buf_resize(buf, len);
        if(ret != SARG_ERR_SUCCESS)
            return ret;
    }


    return SARG_ERR_SUCCESS;
}

static const char *_sarg_opt_type_str[COUNT] = {
    "INT",
    "UINT",
    "DOUBLE",
    "",
    "STRING"
};

/**
 * @brief Prints a help text into the given buffer.
 *
 * The root object has to be initialized with sarg_init before
 * being passed to this function.
 *
 * This function dynamically allocates memory for the given
 * buffer, so that the help text fits into the buffer. outbuf
 * has to be freed manually afterwards.
 *
 * @param root root object for creating help text
 * @param outbuf dynamically allocated output buffer for help text
 *
 * @return SARG_ERR_SUCCESS on success or a SARG_ERR_* code otherwise
 */
int sarg_help_text(sarg_root *root, char **outbuf)
{
    int outlen, linelen,  i, offset, lineoff, ret;
    char *linebuf = NULL;
    const char *type_name;
    *outbuf = NULL;

    // alloc output buffer and tmp buffer
    outlen = 256;
    *outbuf = (char *) malloc(outlen);
    if(!*outbuf)
        return SARG_ERR_ALLOC;

    linelen = 128;
    linebuf = (char *) malloc(linelen);
    if(!linebuf) {
        ret = SARG_ERR_ALLOC;
        goto _sarg_help_text_exit;
    }

    offset = 0;
    ret = _sarg_snprintf(outbuf, &outlen, &offset,
                         "Usage: %s [OPTION]... [ARG]...\n\n", root->name);
    if(ret != SARG_ERR_SUCCESS)
        goto _sarg_help_text_exit;

    for(i = 0; i < root->opt_len; ++i) {
        lineoff = 0;
        linebuf[0] = '\0';
        type_name = _sarg_opt_type_str[root->opts[i].type];
        // create output depending on which options are available
        if(root->opts[i].short_name && root->opts[i].long_name) {
            ret = _sarg_snprintf(&linebuf, &linelen, &lineoff, "  -%s, --%s %s",
                                 root->opts[i].short_name, root->opts[i].long_name, type_name);
            if(ret != SARG_ERR_SUCCESS)
                goto _sarg_help_text_exit;
        } else if(root->opts[i].short_name) {
            ret = _sarg_snprintf(&linebuf, &linelen, &lineoff, "  -%s %s",
                                 root->opts[i].short_name, type_name);
            if(ret != SARG_ERR_SUCCESS)
                goto _sarg_help_text_exit;
        } else if(root->opts[i].long_name) {
            ret = _sarg_snprintf(&linebuf, &linelen, &lineoff, "  --%s %s",
                                 root->opts[i].long_name, type_name);
            if(ret != SARG_ERR_SUCCESS)
                goto _sarg_help_text_exit;
        }

        // print options string into outbuf
        ret = _sarg_snprintf(outbuf, &outlen, &offset, "%-30s", linebuf);
        if(ret != SARG_ERR_SUCCESS)
            goto _sarg_help_text_exit;

        if(root->opts[i].help) {
            ret = _sarg_snprintf(outbuf, &outlen, &offset, "%s", root->opts[i].help);
            if(ret != SARG_ERR_SUCCESS)
                goto _sarg_help_text_exit;
        }

        ret = _sarg_snprintf(outbuf, &outlen, &offset, "\n");
        if(ret != SARG_ERR_SUCCESS)
            goto _sarg_help_text_exit;
    }

    free(linebuf);
    return SARG_ERR_SUCCESS;

_sarg_help_text_exit:
    if(outbuf) {
        free(*outbuf);
        *outbuf = NULL;
    }
    if(linebuf)
        free(linebuf);
    return ret;
}

/**
 * @brief Prints a help text to stdout.
 *
 * @param root root object for creating help text
 *
 * @return SARG_ERR_SUCCESS on success or a SARG_ERR_* code otherwise
 */
int sarg_help_print(sarg_root *root)
{
    char *buf;
    int ret;

    ret = sarg_help_text(root, &buf);
    if(ret != SARG_ERR_SUCCESS)
        return ret;
    printf("%s", buf);
    free(buf);

    return SARG_ERR_SUCCESS;
}

#endif

#ifndef SARG_NO_FILE

#include <stdio.h>
#include <ctype.h>

char *_sarg_trim(char *str)
{
    char *end;

    // Trim leading space
    while(isspace((unsigned char)*str)) str++;

    if(*str == 0)
        return str;

    // Trim trailing space
    end = str + strlen(str) - 1;
    while(end > str && isspace((unsigned char)*end)) end--;

    // Write new null terminator
    *(end + 1) = '\0';

    return str;
}

int _sarg_argv_resize(char ***argv, int *argc)
{
    int argc_tmp = *argc;
    char **argv_tmp = *argv;
    int i;

    *argv = (char **) malloc(sizeof(char *) * argc_tmp * 2);
    if(!(*argv)) {
        *argv = argv_tmp;
        return SARG_ERR_ALLOC;
    }
    *argc = argc_tmp * 2;

    memset(argv, 0, sizeof(char *) * argc_tmp);
    for(i = 0; i < argc_tmp; ++i)
        (*argv)[i] = argv_tmp[i];
    free(argv_tmp);

    return SARG_ERR_SUCCESS;
}

int _sarg_argv_add_arg(const char *fmt, const char *arg, int arglen,
                       char **argv, int *currarg)
{
    int ret;

    argv[*currarg] = (char *) malloc(arglen);
    if(!argv[*currarg])
        return SARG_ERR_ALLOC;

    ret = snprintf(argv[*currarg], arglen, fmt, arg);
    argv[*currarg][arglen - 1] = '\0';
    ++(*currarg);
    if(ret < 0)
        return SARG_ERR_OTHER;

    return SARG_ERR_SUCCESS;
}

int _sarg_argv_add_from_line(const char *line, char ***argv, int *argc,
                             int *currarg)
{
    int diff, ret, arglen;
    char *sep;

    diff = *argc - *currarg;
    if(diff < 2) {
        ret = _sarg_argv_resize(argv, argc);
        if(ret != SARG_ERR_SUCCESS)
            return ret;
    }

    // find separating space
    sep = strchr((char*) line, ' ');
    arglen = strlen(line) + 2;
    if(sep)
        arglen -= strlen(sep);

    // write first arg into argv
    ret = _sarg_argv_add_arg("-%s", line, arglen, *argv, currarg);
    if(ret != SARG_ERR_SUCCESS)
        return ret;

    // write second argument if found into array
    if(sep) {
        sep = _sarg_trim(sep);
        arglen = strlen(sep) + 1;

        _sarg_argv_add_arg("%s", sep, arglen, *argv, currarg);
        if(ret != SARG_ERR_SUCCESS)
            return ret;
    }

    return SARG_ERR_SUCCESS;
}

/**
 * @brief Parse arguments from the given argument file.
 *
 * Options in the file are specified without preceding dashes
 * '-'. If the option expects an additional parameter it has to
 * be separated by a whitespace ' '. One line per option.
 *
 * The arguments in the file have to be specified in the
 * following format:
 *
 * OPTION1 ARG
 * OPTION2 ARG
 * OPTION3
 * ...
 *
 * @param root root object which should be used to parse arguments
 * @param filename file which should be used to read arguments
 *
 * @return SARG_ERR_SUCCESS on success or a SARG_ERR_* code otherwise
 */
 /*
int sarg_parse_file(sarg_root *root, const char *filename)
{
    int argc, currarg, ret, i;
    size_t len;
    FILE *fp = NULL;
    char **argv = NULL;
    char *line = NULL;
    char *line_trim = NULL;

    fp = fopen(filename , "r");
    if(!fp)
        return SARG_ERR_ERRNO;

    currarg = 1;
    argc = 16;
    argv = (char **) malloc(sizeof(char *) * argc);
    if(!argv) {
        ret = SARG_ERR_ALLOC;
        goto _sarg_parse_file_exit;
    }
    memset(argv, 0, sizeof(char *) * argc);

    // convert arguments in file to arg vector
    len = 0;
    while((ret = getline(&line, &len, fp)) != -1) {
        line_trim = _sarg_trim(line);
        if(strlen(line_trim) == 0)
            continue;

        ret = _sarg_argv_add_from_line(line_trim, &argv, &argc, &currarg);
        if(ret != SARG_ERR_SUCCESS)
            goto _sarg_parse_file_exit;
    }

    // parse created arg vector
    ret = sarg_parse(root, (const char **) argv, currarg);
    if(ret != SARG_ERR_SUCCESS)
        goto _sarg_parse_file_exit;

    ret = SARG_ERR_SUCCESS;

_sarg_parse_file_exit:
    if(line)
        free(line);
    if(argv) {
        for(i = 0; i < currarg; ++i) {
            if(argv[i])
                free(argv[i]);
        }
        free(argv);
    }
    if(fp)
        fclose(fp);
    return ret;
}
*/

// TEKKER MOD
// buffer contains single command...
int sarg_parse_command_buffer(sarg_root *root, char *buff, const size_t len)
{
    int argc, currarg, ret, i;
    //size_t len;

    char **argv = NULL;
//    char *line = NULL;
    char *line_trim = NULL;

    currarg = 1;
    argc = 16;
    argv = (char **) malloc(sizeof(char *) * argc);
    if(!argv) {
        ret = SARG_ERR_ALLOC;
        goto _sarg_parse_buff_exit;
    }
    memset(argv, 0, sizeof(char *) * argc);

    // convert arguments in buff str to arg vector

    line_trim = _sarg_trim(buff);
    if(strlen(line_trim) != 0) {
        ret = _sarg_argv_add_from_line(line_trim, &argv, &argc, &currarg);
        if(ret != SARG_ERR_SUCCESS)
            goto _sarg_parse_buff_exit;
    }
    // parse created arg vector
    ret = sarg_parse(root, (const char **) argv, currarg);
    if(ret != SARG_ERR_SUCCESS)
        goto _sarg_parse_buff_exit;
    ret = SARG_ERR_SUCCESS;

_sarg_parse_buff_exit:
    //if(line)
    //    free(line);
    if(argv) {
        for(i = 0; i < currarg; ++i) {
            if(argv[i])
                free(argv[i]);
        }
        free(argv);
    }
    return ret;
}


#endif

#endif
