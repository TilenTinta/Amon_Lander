#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_amon_model.h"

#define NX AMON_MODEL_NX
#define NU AMON_MODEL_NU
#define N AMON_MODEL_N
#define NY AMON_MODEL_NY
#define NYN AMON_MODEL_NYN

#define U_HOVER 90.0

typedef struct
{
    size_t size;
} allocation_header_t;

static size_t heap_current = 0;
static size_t heap_peak = 0;
static size_t heap_total_allocated = 0;
static size_t heap_total_freed = 0;
static size_t malloc_calls = 0;
static size_t calloc_calls = 0;
static size_t realloc_calls = 0;
static size_t free_calls = 0;
static size_t failed_allocs = 0;

void *__real_malloc(size_t size);
void *__real_calloc(size_t nmemb, size_t size);
void __real_free(void *ptr);
void *__real_realloc(void *ptr, size_t size);

static void update_heap_peak(void)
{
    if (heap_current > heap_peak)
        heap_peak = heap_current;
}

static void *profile_alloc(size_t size, int clear)
{
    size_t total_size = size + sizeof(allocation_header_t);
    allocation_header_t *header = (allocation_header_t *)__real_malloc(total_size);

    if (header == NULL)
    {
        failed_allocs++;
        return NULL;
    }

    header->size = size;
    heap_current += size;
    heap_total_allocated += size;
    update_heap_peak();

    void *ptr = (void *)(header + 1);
    if (clear)
        memset(ptr, 0, size);

    return ptr;
}

void *__wrap_malloc(size_t size)
{
    malloc_calls++;
    return profile_alloc(size, 0);
}

void *__wrap_calloc(size_t nmemb, size_t size)
{
    if (size != 0 && nmemb > SIZE_MAX / size)
    {
        failed_allocs++;
        return NULL;
    }

    calloc_calls++;
    return profile_alloc(nmemb * size, 1);
}

void __wrap_free(void *ptr)
{
    if (ptr == NULL)
        return;

    allocation_header_t *header = ((allocation_header_t *)ptr) - 1;
    heap_current -= header->size;
    heap_total_freed += header->size;
    free_calls++;
    __real_free(header);
}

void *__wrap_realloc(void *ptr, size_t size)
{
    if (ptr == NULL)
        return __wrap_malloc(size);

    if (size == 0)
    {
        __wrap_free(ptr);
        return NULL;
    }

    allocation_header_t *old_header = ((allocation_header_t *)ptr) - 1;
    size_t old_size = old_header->size;
    size_t total_size = size + sizeof(allocation_header_t);

    allocation_header_t *new_header = (allocation_header_t *)__real_realloc(old_header, total_size);
    if (new_header == NULL)
    {
        failed_allocs++;
        return NULL;
    }

    new_header->size = size;
    heap_current = heap_current - old_size + size;

    if (size > old_size)
        heap_total_allocated += size - old_size;
    else
        heap_total_freed += old_size - size;

    realloc_calls++;
    update_heap_peak();
    return (void *)(new_header + 1);
}

static void print_heap_stats(const char *label)
{
    printf(
        "%s: heap_current=%zu bytes, heap_peak=%zu bytes, allocated=%zu bytes, "
        "freed=%zu bytes, malloc=%zu, calloc=%zu, realloc=%zu, free=%zu, failed=%zu\n",
        label,
        heap_current,
        heap_peak,
        heap_total_allocated,
        heap_total_freed,
        malloc_calls,
        calloc_calls,
        realloc_calls,
        free_calls,
        failed_allocs);
}

static void set_reference(ocp_nlp_config *nlp_config, ocp_nlp_dims *nlp_dims, ocp_nlp_in *nlp_in)
{
    double yref[NY];
    double yref_e[NYN];
    memset(yref, 0, sizeof(yref));
    memset(yref_e, 0, sizeof(yref_e));

    if (NX > 2)
        yref[2] = 1.0;
    if (NX > 6)
        yref[6] = 1.0;
    if (NY > NX)
        yref[NX] = U_HOVER;

    if (NYN > 2)
        yref_e[2] = 1.0;
    if (NYN > 6)
        yref_e[6] = 1.0;

    for (int i = 0; i < N; i++)
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref);

    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", yref_e);
}

static void set_initial_guess(ocp_nlp_config *nlp_config, ocp_nlp_dims *nlp_dims, ocp_nlp_in *nlp_in, ocp_nlp_out *nlp_out)
{
    double x[NX];
    double u[NU];
    memset(x, 0, sizeof(x));
    memset(u, 0, sizeof(u));

    if (NX > 6)
        x[6] = 1.0;
    if (NU > 0)
        u[0] = U_HOVER;

    for (int i = 0; i <= N; i++)
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "x", x);

    for (int i = 0; i < N; i++)
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "u", u);
}

int main(void)
{
    printf("dims: NX=%d, NU=%d, N=%d, NY=%d, NYN=%d\n", NX, NU, N, NY, NYN);
    print_heap_stats("start");

    amon_model_solver_capsule *capsule = amon_model_acados_create_capsule();
    print_heap_stats("after capsule create");

    int status = amon_model_acados_create(capsule);
    print_heap_stats("after solver create");

    if (status != 0)
    {
        printf("amon_model_acados_create() status=%d\n", status);
        amon_model_acados_free_capsule(capsule);
        print_heap_stats("after capsule free");
        return status;
    }

    ocp_nlp_config *nlp_config = amon_model_acados_get_nlp_config(capsule);
    ocp_nlp_dims *nlp_dims = amon_model_acados_get_nlp_dims(capsule);
    ocp_nlp_in *nlp_in = amon_model_acados_get_nlp_in(capsule);
    ocp_nlp_out *nlp_out = amon_model_acados_get_nlp_out(capsule);

    double x0[NX];
    memset(x0, 0, sizeof(x0));
    if (NX > 6)
        x0[6] = 1.0;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", x0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", x0);

    set_reference(nlp_config, nlp_dims, nlp_in);
    set_initial_guess(nlp_config, nlp_dims, nlp_in, nlp_out);

    print_heap_stats("before first solve");
    status = amon_model_acados_solve(capsule);
    print_heap_stats("after first solve");

    double u0[NU];
    memset(u0, 0, sizeof(u0));
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", u0);

    printf("solve_status=%d, u0=", status);
    for (int i = 0; i < NU; i++)
        printf("%s%.6f", i == 0 ? "" : ",", u0[i]);
    printf("\n");

    int free_status = amon_model_acados_free(capsule);
    printf("amon_model_acados_free() status=%d\n", free_status);
    amon_model_acados_free_capsule(capsule);
    print_heap_stats("after solver free");

    return status;
}
